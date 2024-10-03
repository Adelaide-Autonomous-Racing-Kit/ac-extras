from configparser import ConfigParser
from pathlib import Path
from typing import Dict, List, Union

import numpy as np


def load_vehicle_data(data_path: str) -> Dict:
    config_paths = Path(data_path).glob("*.ini")
    config = create_ini_parser()
    for config_path in config_paths:
        config.read(config_path)
    return format_dictionary_values(config._sections)


def create_ini_parser() -> ConfigParser:
    """
    Creates a configuration parser for AC .ini files
    """
    config = ConfigParser(
        comment_prefixes=("/", ";", "#"),
        inline_comment_prefixes=(";"),
    )
    config.optionxform = lambda x: x
    return config


def format_dictionary_values(dictionary: Dict) -> Dict:
    formatted_data = {}
    for key in dictionary.keys():
        value = dictionary[key]
        if isinstance(value, dict):
            value = format_dictionary_values(value)
        else:
            value = format_value(value)
        formatted_data[key] = value
    return formatted_data


def format_value(value: str) -> Union[str, int, float]:
    value = maybe_remove_inline_comment(value)
    value = maybe_format_a_list_of_values(value)
    return maybe_convert_to_float(value)


def maybe_format_a_list_of_values(value: str) -> Union[str, List]:
    if len(value.split(",")) > 1:
        value = format_list(value)
    return value


def format_list(value: str) -> Union[str, np.array]:
    formatted_values = []
    values = value.split(",")
    for str_value in values:
        formatted_values.append(format_value(str_value))
    if is_list_of_floats(formatted_values):
        return np.array(formatted_values)
    return value


def is_list_of_floats(list: List) -> bool:
    return all([isinstance(value, float) for value in list])


def maybe_remove_inline_comment(value: str) -> str:
    if ";" in value:
        value = value[: value.find(";")]
    return value


def maybe_convert_to_float(value: str) -> Union[str, float]:
    try:
        value = float(value)
    except ValueError:
        # Wasn't a parsable string
        pass
    except TypeError:
        # Wasn't a string
        pass
    return value
