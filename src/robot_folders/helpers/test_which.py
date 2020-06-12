import sys
import subprocess

from helpers import which

def test_is_exe():
    assert which.is_exe(sys.executable)

def test_which_python():
    system_call = subprocess.run(["which", "python3"], capture_output=True)
    assert which.which('python3') == system_call.stdout.strip().decode("utf-8")

def test_which_interpreter():
    assert which.which(sys.executable) == sys.executable

def test_which_none():
    assert which.which('I_dont exist') is None
