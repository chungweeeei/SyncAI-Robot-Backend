"""Service layer: work that is neither transport nor storage.

A service owns a piece of domain behaviour that outlives a single request and
that no one gateway or repository can hold alone -- a background pipeline, the
state of the things it has running, and the record it leaves on disk. Routers
drive them; they know nothing about HTTP.
"""
