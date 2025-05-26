import matplotlib.pyplot as plt
import pandas as pd
f = open("map.csv", "r")
t = f.readline()
x, y = t.split()

df = pd.read_csv("map.csv")
print(df)

df = df.dropna()  # удаляем строки с NaN
plt.plot(df.get(x), df.get(y))
plt.show()

