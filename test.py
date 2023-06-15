# python timer callback
import sched
import time
s = sched.scheduler(time.time, time.sleep)


def print_time1(a='default'):
    print("From print_time1", time.time(), a)
    s.enter(0.001, 2, print_time1, argument=('time1',))


def print_time2(a='default'):
    print("From print_time2", time.time(), a)
    s.enter(0.002, 1, print_time2, kwargs={'a': 'time2'})


def print_time3(a='default'):
    print("From print_time3", time.time(), a)
    s.enterabs(time.time()+5, 0, print_time3, argument=("time3",))


# s.enter(10, 1, print_time)
s.enter(0.001, 2, print_time1, argument=('time1',))
# despite having higher priority, 'keyword' runs after 'positional' as enter() is relative
s.enter(0.002, 1, print_time2, kwargs={'a': 'time2'})
s.enterabs(time.time()+5, 0, print_time3, argument=("time3",))
# s.enterabs(1_650_000_000, 5, print_time, argument=("second enterabs",))
s.run()
