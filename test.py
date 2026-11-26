original  = {"f": 1}

def f(o):
    o["f"] += 1
    return o

print(original, f(original))

print((1,2) + (3,4))