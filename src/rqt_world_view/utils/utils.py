import stoppable_thread
import threading
import subprocess
import time


# Converts meters to milimeters.
def m_to_mm(meters):
    return meters * 1000.0

# Converts millimeters to meters.
def mm_to_m(millimeters):
    return millimeters / 1000.0


# Source: http://stackoverflow.com/questions/2581817/python-subprocess-callback-when-cmd-exits
# Phil's answer.
# I have made a slight change, in that this version uses a StoppableThread (see `stoppable_thread.py`) instaed of a normal one.
def popen_and_call(onExit, *popenArgs, **popenKWArgs):
    """
    Runs a subprocess.Popen, and then calls the function onExit when the
    subprocess completes.

    Use it exactly the way you'd normally use subprocess.Popen, except include a
    callable to execute as the first argument. onExit is a callable object, and
    *popenArgs and **popenKWArgs are simply passed up to subprocess.Popen.
    """
    def runInThread(onExit, popenArgs, popenKWArgs):
        proc = subprocess.Popen(*popenArgs, **popenKWArgs)

        # Keep running while the subprocess is alive.
        while proc.poll() == None:
            if threading.current_thread().stopped():
                # We need to stop, kill the subprocess.
                proc.terminate()

            # Sleep for a tenth of a second.
            time.sleep(0.1)

        # Call the exit callback.
        onExit()
        return

    thread = stoppable_thread.StoppableThread(target=runInThread,
                              args=(onExit, popenArgs, popenKWArgs))
    thread.start()

    return thread # returns immediately after the thread starts
