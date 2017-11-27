import time
import threading

def task1():
    while True:
        time.sleep(1)
        print("Task1 is running")


def task2():
    while True:
        time.sleep(2)
        print("Task2 is running")
        
threads = []
t = threading.Thread(target=task1)
threads.append(t)
t.start()

t = threading.Thread(target=task2)
threads.append(t)
t.start()