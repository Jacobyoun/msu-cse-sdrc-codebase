import os
from flask import Flask
# from flask_socketio import SocketIO
from flask_failsafe import failsafe

# socketio = SocketIO()

@failsafe
def create_app():
	app = Flask(__name__)
	# socketio.init_app(app)
	with app.app_context():
		from . import routes
		return app
