from flask import Flask, render_template
from flask import request
import backend as be
import os

app = Flask(__name__)


# @app.route("/")
# def index():
#     return render_template('default_guifrontend.html')

@app.route("/", methods=["GET", "POST"])
def plot():
    if request.method == "POST":
        capture_frontend_req = request.form.to_dict()
        startInput = capture_frontend_req['start_holder']
        endInput = capture_frontend_req['end_holder']
        typeButton = capture_frontend_req['buttonType']
        print(typeButton)
        if typeButton == "walkRouting":
            # os.remove("templates/gui_frontend.html")
            data = be.walkingBackEnd(startInput, endInput)
            if type(data) == list:
                dist = data[0]
                time = data[1]
                return render_template('user.html', startInput=startInput, endInput=endInput, dist=dist, time=time)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
        elif typeButton == "driveRouting":
            # os.remove("templates/gui_frontend.html")
            data = be.drivingBackEnd(startInput, endInput)
            print(data)
            if type(data) == list:
                distDrive = data[0]
                timeDrive = data[1]
                return render_template('user.html', startInput=startInput, endInput=endInput, dist=distDrive,
                                       time=timeDrive)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
        elif typeButton == "busRouting":
            pass
        elif typeButton == "lrtRouting":
            pass
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
