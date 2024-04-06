import asyncio
import time

prefix = "a"
async def count_up():
    global prefix
    p = prefix
    prefix = chr(ord(prefix)+1)
    c = 1
    while c <= 10:
        print(p, ":", c)
        c+=1
        await asyncio.sleep(1)
async def run_them():
    tasks = set()
    t1 = asyncio.create_task(count_up())
    tasks.add(t1)
    t1.add_done_callback(tasks.discard)
    t2 = asyncio.create_task(count_up())
    tasks.add(t2)
    t2.add_done_callback(tasks.discard)
    await t2

    # count_up()
    # count_up()

if __name__=='__main__':
    # print(chr(ord('a')+1))
    asyncio.run(run_them())
    