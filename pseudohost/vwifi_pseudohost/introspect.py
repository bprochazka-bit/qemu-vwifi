#
# vwifi-pseudohost — argparse introspection for machine-readable config
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The pseudo-host and pseudo-AP launchers each describe their command-line
# surface with an argparse parser. Rather than hand-maintain a second copy
# of that surface for tools that drive these binaries (Nyxus builds a web
# form per device type, for instance), we introspect the parser itself and
# emit a JSON descriptor. The parser stays the single source of truth: add
# an argument and it shows up in the dumped config for free.
#
# The descriptor shape (stable — external tools parse it):
#
#   {
#     "device_type": "wifi-host",
#     "binary": "vwifi-pseudohost",
#     "description": "...",
#     "params": [
#       {
#         "name": "essid",          # the dest, a stable key
#         "flags": ["--essid"],     # [] for a positional
#         "positional": false,
#         "type": "str",            # str | int | float | bool
#         "required": true,
#         "default": null,
#         "choices": ["a", "b"],    # or null
#         "help": "...",
#         "metavar": "ESSID"        # or null
#       },
#       ...
#     ],
#     "extras": { ... },            # device-type-specific enrichment
#     "constraints": [              # cross-field validation rules
#       {
#         "requires": "passphrase",         # this param must be non-empty
#         "when": {"param": "encryption",   # ...when another param
#                  "equals": "wpa2"},       #    equals this value
#         "message": "WPA2 needs a passphrase."   # shown to the user
#       },
#       ...
#     ]
#   }
#
# Constraints let a launcher declare a rule its own argument parser
# enforces (e.g. pseudoap rejects `--encryption wpa2` with no passphrase)
# so a driver can check it up front and show the message, instead of only
# discovering it when the launcher exits.
#
import argparse
import json


def _type_name(action):
    """Map an argparse action to one of str/int/float/bool."""
    if isinstance(action, (argparse._StoreTrueAction,
                           argparse._StoreFalseAction)):
        return "bool"
    t = action.type
    if t is int:
        return "int"
    if t is float:
        return "float"
    if t is str or t is None:
        return "str"
    # A custom converter (e.g. a lambda or a named function). Its own name
    # is more useful than "str" for a human reading the dump, but callers
    # should treat anything they don't recognize as a free-text string.
    return getattr(t, "__name__", "str")


def _jsonable(value):
    """Coerce an argparse default to something JSON can carry."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def parser_to_params(parser):
    """Turn an ArgumentParser's arguments into a list of param descriptors.

    The automatically-added help action and the config-dump switch are
    skipped; everything else the parser accepts is described.
    """
    params = []
    for action in parser._actions:
        if isinstance(action, argparse._HelpAction):
            continue
        if action.dest in ("help", "dump_config"):
            continue
        flags = list(action.option_strings)
        positional = not flags
        params.append({
            "name": action.dest,
            "flags": flags,
            "positional": positional,
            "type": _type_name(action),
            "required": bool(action.required) or positional,
            "default": _jsonable(action.default),
            "choices": list(action.choices) if action.choices else None,
            "help": action.help or "",
            "metavar": (action.metavar if isinstance(action.metavar, str)
                        else None),
        })
    return params


def build_descriptor(parser, *, device_type, binary, extras=None,
                     constraints=None):
    """Assemble the full JSON-able descriptor for a launcher."""
    return {
        "device_type": device_type,
        "binary": binary,
        "description": (parser.description or "").strip(),
        "params": parser_to_params(parser),
        "extras": extras or {},
        "constraints": constraints or [],
    }


def dump_descriptor(parser, *, device_type, binary, extras=None,
                    constraints=None):
    """Serialize the descriptor as pretty JSON (what --dump-config prints)."""
    return json.dumps(
        build_descriptor(parser, device_type=device_type, binary=binary,
                         extras=extras, constraints=constraints),
        indent=2, sort_keys=False)
