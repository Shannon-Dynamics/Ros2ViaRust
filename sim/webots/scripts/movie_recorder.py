#!/usr/bin/env python3
"""Record the Webots 3D view to a movie file, as an extern Supervisor controller.

This exists so that a run can produce visual evidence without touching the official
`webots_ros2_turtlebot` driver or the official `Ros2Supervisor`. It attaches to the
separate `movie_recorder` Robot node of the world, which is a supervisor and has no
devices, and it only calls the Webots movie API:

    movieStartRecording(filename, width, height, codec, quality, acceleration, caption)
    movieStopRecording()
    movieIsReady()
    movieFailed()

Those are the signatures of the installed Webots Python controller library
(`$WEBOTS_HOME/lib/controller/python/controller/supervisor.py`), not guessed ones.

Recording renders the main 3D view, so the simulation has to run with a GUI. It does
nothing else: it never moves the robot, never spawns nodes and never ends the simulation.

Configuration is by environment variable, because an extern controller started by
`webots-controller` takes no arguments of its own:

    MOVIE_FILE          output path, required, created by Webots
    MOVIE_SECONDS       how long to record, in simulated seconds (default 40)
    MOVIE_START_DELAY   simulated seconds to wait before starting (default 6), so that the
                        ros2_control spawners are done and the first frame is not a half
                        loaded world
    MOVIE_WIDTH         default 1280
    MOVIE_HEIGHT        default 720
    MOVIE_QUALITY       1 to 100, default 90
    MOVIE_ACCELERATION  playback speed up factor, default 1
    MOVIE_STATUS_FILE   JSON status written when encoding finished; the same path with
                        ".started" appended is touched the moment recording begins, which
                        is what a run script waits for
"""

import json
import os
import sys

from controller import Supervisor


def env_int(name, default):
    return int(os.environ.get(name, default))


def main():
    movie = os.environ.get('MOVIE_FILE')
    if not movie:
        print('movie_recorder: MOVIE_FILE is not set, nothing to record', flush=True)
        return 1
    movie = os.path.abspath(movie)
    os.makedirs(os.path.dirname(movie), exist_ok=True)

    seconds = float(os.environ.get('MOVIE_SECONDS', '40'))
    delay = float(os.environ.get('MOVIE_START_DELAY', '6'))
    width = env_int('MOVIE_WIDTH', 1280)
    height = env_int('MOVIE_HEIGHT', 720)
    quality = env_int('MOVIE_QUALITY', 90)
    acceleration = env_int('MOVIE_ACCELERATION', 1)
    status_file = os.environ.get('MOVIE_STATUS_FILE')

    sup = Supervisor()
    dt = int(sup.getBasicTimeStep())
    t_connect = sup.getTime()
    print(f'movie_recorder: connected at sim t={t_connect:.3f}, basic time step {dt} ms, '
          f'will record {seconds:.1f} s to {movie}', flush=True)

    started = stopped = False
    t_start = t_stop = None
    status = {
        'movie_file': movie,
        'width': width,
        'height': height,
        'quality': quality,
        'acceleration': acceleration,
        'codec': 0,
        'requested_seconds': seconds,
        'start_delay_seconds': delay,
        'basic_time_step_ms': dt,
    }

    while sup.step(dt) != -1:
        now = sup.getTime()

        if not started and now - t_connect >= delay:
            # codec 0 is the Webots default, caption False keeps the frame clean.
            sup.movieStartRecording(movie, width, height, 0, quality, acceleration, False)
            started, t_start = True, now
            status['started_sim_time'] = now
            print(f'movie_recorder: recording started at sim t={now:.3f}', flush=True)
            if status_file:
                with open(status_file + '.started', 'w') as fh:
                    fh.write(f'{now}\n')

        if started and not stopped and now - t_start >= seconds:
            sup.movieStopRecording()
            stopped, t_stop = True, now
            status['stopped_sim_time'] = now
            status['recorded_sim_seconds'] = now - t_start
            print(f'movie_recorder: recording stopped at sim t={now:.3f}, encoding', flush=True)

        # movieIsReady() is also true before anything was recorded, so it is only trusted
        # after a stop, and only after giving the encoder a moment to pick the job up.
        if stopped and now - t_stop >= 1.0 and sup.movieIsReady():
            status['failed'] = bool(sup.movieFailed())
            status['exists'] = os.path.exists(movie)
            status['size_bytes'] = os.path.getsize(movie) if status['exists'] else 0
            status['finished_sim_time'] = now
            print(f'movie_recorder: done, failed={status["failed"]} '
                  f'exists={status["exists"]} bytes={status["size_bytes"]}', flush=True)
            if status_file:
                with open(status_file, 'w') as fh:
                    json.dump(status, fh, indent=2)
                    fh.write('\n')
            return 0

    # The simulation ended before the movie was finished; say so instead of pretending.
    status['aborted'] = True
    status['started'] = started
    status['stopped'] = stopped
    status['exists'] = os.path.exists(movie)
    status['size_bytes'] = os.path.getsize(movie) if status['exists'] else 0
    print('movie_recorder: simulation ended before the movie was finished', flush=True)
    if status_file:
        with open(status_file, 'w') as fh:
            json.dump(status, fh, indent=2)
            fh.write('\n')
    return 2


if __name__ == '__main__':
    sys.exit(main())
