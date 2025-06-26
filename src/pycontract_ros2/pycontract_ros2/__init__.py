"""PyContract helper utilities for working with ROS 2 messages.

Currently provides utilities to make Python structural-pattern matching
(PEP 634) work with ROS 2 generator message classes by adding the
``__match_args__`` attribute that the pattern-matching engine looks for.
"""
from __future__ import annotations

import importlib
from types import ModuleType
from typing import Type

__all__ = [
    "enable_pattern_matching_for_ros2_message",
    "enable_pattern_matching_for_ros2_package",
]


def _add_match_args(msg_cls: Type[object]) -> None:
    """Internal helper – ensure *msg_cls* has a suitable ``__match_args__``.

    The ROS 2 Python generator populates ``_fields_and_field_types`` with the
    public message field names.  If that attribute is present we use it;
    otherwise we fall back to stripping the leading underscore from each slot
    entry (old generator behaviour).
    """
    if hasattr(msg_cls, "__match_args__"):
        # Nothing to do – already patched.
        return

    if hasattr(msg_cls, "_fields_and_field_types"):
        field_names = tuple(msg_cls._fields_and_field_types.keys())  # type: ignore[attr-defined]
    else:
        field_names = tuple(slot.lstrip("_") for slot in getattr(msg_cls, "__slots__", ()))

    msg_cls.__match_args__ = field_names  # type: ignore[assignment]


def enable_pattern_matching_for_ros2_message(msg_cls: Type[object]) -> Type[object]:
    """Patch a single ROS 2 *msg_cls* so it works with ``match``/``case``.

    Returns the same class object for convenience so the call can be used in a
    fluent style: ``A = enable_pattern_matching_for_ros2_message(A)``.
    """
    _add_match_args(msg_cls)
    return msg_cls


def enable_pattern_matching_for_ros2_package(pkg_name: str) -> None:
    """Patch every generated message class found in *pkg_name*.msg.*.

    The generator does not define ``__all__``, so we scan the module attributes
    and look for objects that have the marker attribute
    ``_fields_and_field_types`` that is present on all generated message
    classes.
    """
    msg_module: ModuleType = importlib.import_module(f"{pkg_name}.msg")
    for attr_name in dir(msg_module):
        attr = getattr(msg_module, attr_name)
        if isinstance(attr, type) and hasattr(attr, "_fields_and_field_types"):
            _add_match_args(attr)
