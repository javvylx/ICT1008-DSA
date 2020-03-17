from flask import Flask, render_template, request

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('user.html')

# @app.route("/", methods=["GET", "POST"])
# def need_input():
#     for key, value in request.form.items():
#         print("key: {0}, value: {1}".format(key, value))
#
# @app.route("/form", methods=["GET"])
# def get_form():
#     return render_template('user.html')

if __name__ == '__main__':
    app.run(debug=True)
