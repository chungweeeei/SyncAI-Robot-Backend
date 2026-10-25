"""The map routes, one module per area; ``routers/map.py`` composes them.

Each area module has ``register(map_router, ctx)``: its routes are closures
over the ``MapRouteContext`` it is handed, as they were over
``init_map_router``'s arguments before the split.
"""
