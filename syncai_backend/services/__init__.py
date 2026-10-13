"""Service layer: work that is neither transport nor storage.

Domain behaviour that outlives a request and that no single gateway or
repository can hold. Routers drive services; services know nothing about HTTP.
"""
