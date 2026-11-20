"""The /docs page: FastAPI's Swagger UI, set up for this API.

The page itself is `get_swagger_ui_html`'s; what is added here is a handful of
Swagger UI settings and one stylesheet. Every rule in it selects on Swagger
UI's own class names and ids, which a release may rename: the rule then stops
matching and the page falls back to the stock look, with nothing broken.
"""

from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.responses import HTMLResponse


SWAGGER_UI_PARAMETERS = {
    # Some eighty operations: the page opens as a list of its tags, not all of
    # them unfolded. A deep link still opens the operation it names.
    "docExpansion": "none",
    # The token is kept in the browser's localStorage, so a reload does not
    # sign the reader out; it still expires on its own (JWT_EXPIRE_SECONDS).
    "persistAuthorization": True,
    # Some calls wait on a ROS service for a minute; say how long one took.
    "displayRequestDuration": True,
    # tryItOutEnabled stays off on purpose: the extra click is the one between
    # reading about an estop or a map delete and being able to send it.
}

_STYLE = """<style>
    /* Sign-in: Swagger UI draws an OAuth2 password dialog with the client's
       own credentials beside the account's, and has no setting that leaves
       them out. This server reads none of them, so their rows are hidden:
       each is a .wrapper whose own label names the field. */
    .auth-container .wrapper:has(> label[for="password_type"]),
    .auth-container .wrapper:has(> label[for="client_id_password"]),
    .auth-container .wrapper:has(> label[for="client_secret_password"]) {
        display: none;
    }
    /* Nor does it have scopes to explain. */
    .swagger-ui .scope-def {
        display: none;
    }

    /* The Authorize bar stays in reach however far the page is scrolled. Its
       z-index also carries the sign-in dialog, which is drawn inside it. */
    .swagger-ui .scheme-container {
        position: sticky;
        top: 0;
        z-index: 10;
        padding: 12px 0;
    }

    /* Signed in or not, at a glance: a solid button and green locks. */
    .swagger-ui .btn.authorize.locked {
        background-color: #49cc90;
        color: #fff;
    }
    .swagger-ui .btn.authorize.locked svg {
        color: #fff;
    }
    .swagger-ui .authorization__btn .locked {
        color: #49cc90;
    }

    /* A line of 1460px is long to read across; less air above the first tag. */
    .swagger-ui .wrapper {
        max-width: 1200px;
    }
    .swagger-ui .info {
        margin: 28px 0 20px;
    }
    </style>"""


def render_docs_page(openapi_url: str, title: str) -> HTMLResponse:
    page = get_swagger_ui_html(
        openapi_url=openapi_url,
        title=title,
        swagger_ui_parameters=SWAGGER_UI_PARAMETERS,
    )
    return HTMLResponse(
        page.body.decode().replace("</head>", _STYLE + "\n    </head>", 1)
    )
