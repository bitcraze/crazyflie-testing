from functools import wraps
from time import sleep

def reboot_wrapper(func):
    """Decorator to reboot the device if an exception is raised in the decorated function."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            func(*args, **kwargs)
        except:
            sleep(5)  # wait for assert-induced reboot into error state
            kwargs['test_setup'].device.reboot()
            sleep(5)  # wait for reboot
            raise
    return wrapper
