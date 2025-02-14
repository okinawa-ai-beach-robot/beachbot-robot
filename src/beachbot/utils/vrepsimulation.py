import concurrent.futures as futures

vrep_exec = futures.ThreadPoolExecutor(1)
def vrep_schedule_work(func, *args, **kwargs):
        vrep_exec.submit(func, *args, **kwargs).result()
        
def vrep(func):
    def schedule_work(*args, **kwargs):
        return vrep_exec.submit(func, *args, **kwargs).result()
    return schedule_work
