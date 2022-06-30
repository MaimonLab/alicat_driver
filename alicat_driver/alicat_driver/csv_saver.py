#!/usr/bin/env python3

import csv
from bowltracker_interfaces.msg import FlyEllipsoid
import numpy as np


class CSVWriter:
    def __init__(self, filename=None):

        if filename is None:
            raise FileNotFoundError("No filename provided")
        self.filename = filename

        self.header_initialized = False

    def save_message(self, msg_to_save):

        header, data_row = self.flatten_message(msg_to_save)

        if not self.header_initialized:

            with open(self.filename, "w") as file:
                writer = csv.writer(file)
                writer.writerow(header)

            self.header_initialized = True

        with open(self.filename, "a+") as file:
            writer = csv.writer(file)
            writer.writerow(data_row)

    def flatten_message(self, message):
        """Top level flattening of message, ensuring that the header and data row start with timestamp and frame_id.
        This calls flatten_recursive, which will unpack the message depth-first.
        The recursive flattening is helpful since we do not know the depth of the messages a priori.
        """

        header = []
        data_row = []
        if hasattr(message, "header"):
            timestamp = int(
                message.header.stamp.sec * 1e9 + message.header.stamp.nanosec
            )
            header.append("timestamp")
            data_row.append(timestamp)

            header.append("frame_id")
            data_row.append(message.header.frame_id)

        header, data_row = self.flatten_recursive(message, header, data_row)

        return header, data_row

    def flatten_recursive(
        self, partial_message, header, data_row, parent_field_name=None
    ):
        """flattens the message by finding all its fields, adding it to the list if float, int, string or array.
        If the field is a sub-message, it will call flatten_recursive on the field.
        the prepend_field builds up the header name, ensuring that the header contains information of prior unpackings.
        """

        # loop over the fields of the message
        for sub_field, _ in partial_message.get_fields_and_field_types().items():

            # if the entry is a header, it is taken care of by the flatten_message method
            if sub_field == "header":
                continue

            # get the item belonging to the sub_field name
            sub_item = getattr(partial_message, sub_field)

            # The column name will be an accumulation of field names.
            # if prepend_field is None, we're at the top level and we'll name it the sub_field
            if parent_field_name is None:
                accumulated_field_name = f"{sub_field}"
            else:
                accumulated_field_name = f"{parent_field_name}_{sub_field}"

            # if the item is a float, int or string, we can simply append
            if isinstance(sub_item, (int, float, str)):
                header.append(accumulated_field_name)
                data_row.append(sub_item)

            # if the item is an array, we must unpack it.
            # we'll give it a name of fieldname_0, fieldname_1, ...
            elif type(sub_item) == np.ndarray:
                data_row.extend(sub_item.tolist())
                for i in range(len(sub_item)):
                    header.append(f"{accumulated_field_name}_{i}")

            elif type(sub_item) == list:
                # breakpoint()

                if hasattr(sub_item[0], "get_fields_and_field_types"):
                    for i, list_item in enumerate(sub_item):
                        header, data_row = self.flatten_recursive(
                            list_item, header, data_row, f"{accumulated_field_name}_{i}"
                        )
                else:
                    data_row.extend(sub_item)
                    for i in range(len(sub_item)):
                        header.append(f"{accumulated_field_name}_{i}")

            # if the item has fields, we need to flatten it more. We call it with the accumulated field name
            elif hasattr(sub_item, "get_fields_and_field_types"):

                header, data_row = self.flatten_recursive(
                    sub_item, header, data_row, accumulated_field_name
                )

        return header, data_row


def main():
    filename = "/home/maimon/temp/example_csv_saver.csv"

    header = ["x", "y", "z"]
    csv_saver = CSVWriter(filename=filename, header=header)

    for i in range(10):
        msg = FlyEllipsoid(x=float(i), arena_name="bla")
        csv_saver.save_message(msg)


if __name__ == "__main__":
    main()