import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def main():

    df = pd.read_csv("/home/maimon/temp/alicat_device_measured_flowrate_5.csv")
    t_min = np.around(df["timestamp"].iloc[0], -9)
    t_max = np.around(df["timestamp"].iloc[-1], -9)
    df["wave"] = np.cos(df["timestamp"] / 1e9)
    df.set_index("timestamp", inplace=True)

    t_goal = np.arange(t_min, t_max, 2e8).astype(int)

    df_new = df.reindex(index=t_goal, method="nearest")

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    df.plot(marker=".", y="wave", ax=ax, label="original")
    df_new.plot(linewidth=0, marker=".", y="wave", ax=ax, label="interp")
    ax.legend()

    plt.show()


if __name__ == "__main__":
    main()
