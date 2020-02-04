import sys

while True:
    try:
        n = float(input())
        n = n + 100
        print(n)
    except ValueError:
        print(0)

