from flask import Flask, render_template, request

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('user.html')

@app.route("/", methods=["GET", "POST"])
def need_input():
    for key, value in request.form.items():
        start = request.form.items.get("start", "")
        end = request.form.items.get("end", "")
        print("key: {0}, value: {1}".format(key, value))
    return start,end

@app.route("/form", methods=["GET"])
def get_form():
    return render_template('user.html')

if __name__ == '__main__':
    app.run(debug=True)
