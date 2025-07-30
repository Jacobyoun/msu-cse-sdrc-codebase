import os

from flask import jsonify
from flask_app import create_app

app = create_app()
#responsible for starting the website
if __name__ == "__main__":
	app.run(host='0.0.0.0', port=int(os.environ.get("PORT", 8080)), use_reloader=True, debug=True)
