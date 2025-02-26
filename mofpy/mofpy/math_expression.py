from functools import partial
import re

import sympy

from .shared import Shared


class MathExpression:

    def __init__(self):
        pass

    @staticmethod
    def expressions(values: dict, named_buttons, named_axes) -> dict:
        for key, value in values.items():
            success = True
            if isinstance(value, dict):
                values[key], success = MathExpression.expressions(value, named_buttons, named_axes)
            if isinstance(value, list):
                for i in range(len(value)):
                    if isinstance(value[i], str):
                        value[i], success = MathExpression.__expression__(
                            value[i], named_buttons, named_axes
                        )
                    elif isinstance(value[i], dict):
                        values[key][i], success = MathExpression.expressions(
                            value[i], named_buttons, named_axes
                        )
            if isinstance(value, str):
                values[key], success = MathExpression.__expression__(
                    value, named_buttons, named_axes
                )
            if not success:
                return values, False
        return values, True

    @staticmethod
    def __shared__(key):
        return Shared.get(str(key))

    @staticmethod
    def __axis__(axis, named_axes):
        axis_str = str(axis)
        return named_axes[axis_str].value if axis_str in named_axes else 0

    @staticmethod
    def __button__(button, named_buttons):
        return 1 if named_buttons[str(button)].value else 0

    @staticmethod
    def __expression__(value, named_buttons, named_axes):
        variables = {
            "shared": MathExpression.__shared__,
            "axis": partial(MathExpression.__axis__, named_axes=named_axes),
            "button": partial(MathExpression.__button__, named_buttons=named_buttons),
        }

        match = re.match(r"\${(.+)}", value)
        if match:
            try:
                expr = sympy.sympify(match.group(1), locals=variables)
                return expr, True
            except (sympy.SympifyError, ValueError, AttributeError):
                return value, False
        return value, True
