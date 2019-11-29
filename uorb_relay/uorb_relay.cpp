
/****************************************************************************
 *
 *   Copyright (C) 2019 Peter van der Perk. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uorb_relay.cpp
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#include "uorb_relay.h"
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/airspeed.h>

px4::AppState UorbRelay::appState;

int UorbRelay::main()
{
    appState.setRunning(true);

    /* subscribe to debug_key_value topic */
    int debug_key_value_fd = orb_subscribe(ORB_ID(debug_key_value));
    /* limit the update rate to 5 Hz */
    orb_set_interval(debug_key_value_fd, 200);

    struct airspeed_s dkv;
    memset(&dkv, 0, sizeof(dkv));
    orb_advert_t dkv_pub = orb_advertise(ORB_ID(airspeed), &dkv);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = debug_key_value_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = debug_key_value_fd,   .events = POLLIN },
         */
    };

    int error_counter = 0;
    int i = 0;
    
    struct timespec begin;

    while (!appState.exitRequested()) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);
        
        if(verbose)
        {                                  
            struct timespec end;
            px4_clock_gettime(CLOCK_REALTIME, &end);
            double elapsed_secs = double(end.tv_sec - begin.tv_sec) + double(end.tv_nsec - begin.tv_nsec)/double(1000000000);
            PX4_INFO("%.06f seconds", elapsed_secs);
            px4_clock_gettime(CLOCK_REALTIME, &begin);
        }

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_INFO("Got no data within a second");
        } else if (poll_ret < 0) {
             /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }
            error_counter++;
        } else {
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct debug_key_value_s raw;
                /* copy key value raw data into local buffer */
                if (orb_copy(ORB_ID(debug_key_value), debug_key_value_fd, &raw) != 0) {
                    PX4_ERR("[%d]Error calling orb copy ... ", i);
                }

                if(strncmp(raw.key, "ROS", 3) == 0) {
                    dkv.timestamp = raw.timestamp;

                    orb_publish(ORB_ID(airspeed), dkv_pub, &dkv);

                    if(verbose)
                    {
                        PX4_INFO("Seq nr: \t%i Timestamp: \t%lld",
                            (int)raw.value,
                            raw.timestamp);
                    }
                }
            }
        }
    }

    appState.setRunning(false);
    return 0;
}
