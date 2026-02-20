import numpy as np
import sys
import pandas as pd
import matplotlib.pyplot as plt
import time

arguments = sys.argv
if len(arguments) > 1:
    print("We have arguments!")
    print()
    run_time = int(arguments[1])
    print(f"We are going to run this script for {run_time} seconds")
    for i in range(run_time):
        time.sleep(1)
        print(i+1)
    print("Goodbye") 
else:
    print("no arguments :(")
# print(arguments)
