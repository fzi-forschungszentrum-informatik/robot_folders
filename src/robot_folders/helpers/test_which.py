import sys

from helpers import which

def test_is_exe():
    assert which.is_exe(sys.executable)

def test_which_python():
    assert which.which('python') == sys.executable

def test_which_interpreter():
    assert which.which(sys.executable) == sys.executable

def test_which_none():
    assert which.which('I_dont exist') == None

