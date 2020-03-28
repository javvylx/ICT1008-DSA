from flask import Flask, render_template
from flask import request
import backend as be

app = Flask(__name__)


@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":

        capture_frontend_req = request.form.to_dict()
        startInput = capture_frontend_req['start_holder']
        endInput = capture_frontend_req['end_holder']
        # typeButton = capture_frontend_req['buttonType']

        data = be.walkingBackEnd(startInput, endInput)
        dist = data[0]
        time = data[1]
        return render_template('user.html', startInput=startInput, endInput=endInput, dist=dist, time=time)

    # Need help bro for the bottom part

        # if type(data) == bytearray
        #     dist = data[0]
        #     time = data[1]
        #     return render_template('user.html', startInput=startInput, endInput=endInput, dist=dist, time=time)
        # elif type(data) == str:
        #     data = data
        #     return render_template('user.html', startInput=startInput, endInput=endInput, data=data)

        # if typeButton == "walkRouting":
        #     data = be.walkingBackEnd(startInput, endInput)
        #     return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
        # elif typeButton == "busRouting":
        #     data2 = be.drivingBackEnd(startInput, endInput)
        #     return render_template('user.html', startInput=startInput, endInput=endInput, data2=data2)
        # elif typeButton == "lrtRouting":
        #     data3 = be.lrtBackEnd(startInput, endInput)
        #     return render_template('user.html', startInput=startInput, endInput=endInput, data3=data3)

    return render_template('user.html')


if __name__ == '__main__':
    app.run(debug=True)
