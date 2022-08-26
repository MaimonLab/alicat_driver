# alicat airflow driver

This package is a ROS2 wrapper around the alicat python library for serial communcation to the alicat massflow controller.

## Installation

Clone the package and run the install script:

    cd ~/maimon_ws/src
    git clone git@github.com:MaimonLab/alicat_driver.git
    cd alicat_driver
    git checkout install-script
    ./install_dependencies.sh

## Use case

The alicat node can receive flowrate either through a service, or through a `goal_flowrate` topic. If you change the flowrate only sparsely, I recommend using the service since is more reliable.
The alicat will publish its measured flowrate once a second regardless, so you can always inspect the current flowrate on the device.

## Example

The example launch file starts the alicat airflow server and example client.

    ros2 launch alicat_device example_server.launch.py

This will prompt you to pick one of the config files with parameters. You can open a device by:

    1. A serial number (e.g. FT4TF9NT)
    2. A serial port (e.g. /dev/ttyUSB0)
    3. Neither, the node will try to find a connected device

Using the serial number is most robust as ports are subject to change. You can find out the serial number and port of your device by plugging it in and running:

    ros2 run alicat_device list_ports

Look for a device manufactured by FTDI.

Similarly, you can launch a publisher/subscriber example:

    ros2 launch alicat_device example_subscriber.launch.py

# Alicat timestamps

Since the alicat publishes and saves flowrate data on it's own timer, the measurement timestamps don't directly correspond to timestamps of messages and data of other nodes. We can interpolate the alicat data to a dataframe with the `merge_dataframes_mismatching_timestamps` function:

```

def merge_dataframes_mismatching_timestamps(
    df_main, df_alicat, reset_index=True, method="index"
):
    """Method options:
    method="index", linearly interpolate between main timestamps
    method="nearest", interpoalte using nearest values
    """
    df_alicat = df_alicat.copy() # ensure we don't modify original dataframes
    df_main = df_main.copy()

    df_alicat.set_index("timestamp", inplace=True) # to timestamp index to interpolate on
    df_main.set_index("timestamp", inplace=True)

    df_main = pd.concat([df_main, df_alicat]).sort_index()

    df_main.interpolate(method=method, inplace=True, limit_area="inside")

    df_main.drop(df_alicat.index, inplace=True) # remove alicat dataframes

    if reset_index: # if you don't want to keep the timestamps as the index
        df_main.reset_index(inplace=True)

    return df_main
```

### Interpolation example

```
main_data = {
    "timestamp": [0, 1, 2, 3, 4, 5],
    "x_position": [3.2, 3.6, 3.8, 2.4, 2.8, 3.2],
}
alicat_data = { "timestamp": [1.5, 4.5], "flowrate": [2.0, 1.0] }

df_main = pd.DataFrame(main_data)
df_alicat = pd.DataFrame(alicat_data)

df_main = merge_dataframes_mismatching_timestamps(df_main, df_alicat)

fig, ax = plt.subplots(1, 1, figsize=(8, 8))
df_main.plot(marker=".", x="timestamp", y="x_position", ax=ax, label="x_position")
df_main.plot( marker=".", x="timestamp", y="flowrate", ax=ax, label="interpolated flowrate")
df_alicat.plot(
    linewidth=0,
    x="timestamp",
    marker=".",
    y="flowrate",
    ax=ax,
    label="alicat flowrate",
)
ax.legend(); plt.show()
```
