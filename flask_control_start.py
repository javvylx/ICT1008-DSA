from flask import Flask, render_template
from flask import request

app = Flask(__name__)

@app.route("/", methods=["GET", "POST"])
def index():
    start()
    stop()
    return render_template('user.html')


def start():
    if request.method == "POST":  # check that data is coming from POST request
        capture_frontend_req = request.form.to_dict()
        print(capture_frontend_req['start_holder'])  # extract based on html form input id name
        startInput = capture_frontend_req['start_holder']  # if you choose to assign to variable
        # endInput = capture_frontend_req['end_holder']

        return startInput


def stop():
    if request.method == "POST":  # check that data is coming from POST request
        capture_frontend_req = request.form.to_dict()
        print(capture_frontend_req['end_holder'])  # same for 2nd form

        # startInput = capture_frontend_req['start_holder']  # if you choose to assign to variable
        endInput = capture_frontend_req['end_holder']

        return endInput

if __name__ == '__main__':
    app.run(debug=True)
