from flask import Flask, render_template
from flask import request
import backend as be
import datetime
import os

app = Flask(__name__)


# @app.route("/")
# def index():
#     return render_template('default_guifrontend.html')

currentDT = datetime.datetime.now()
current_time = int(currentDT.strftime("%H%M"))
print(current_time)
# current_time = 1200


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
                endtime = str(round(current_time + float(time)))

                return render_template('user.html', startInput=startInput, endInput=endInput, dist=dist, time=time,
                                       current_time=current_time, endtime=endtime)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
        elif typeButton == "driveRouting":
            # os.remove("templates/gui_frontend.html")
            data = be.drivingBackEnd(startInput, endInput)
            print(data)
            if type(data) == list:
                distDrive = data[0]
                timeDrive = data[1]
                endtime = str(round(current_time + float(timeDrive)))
                return render_template('user.html', startInput=startInput, endInput=endInput, dist=distDrive,
                                       time=timeDrive, current_time=current_time, endtime=endtime)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
        elif typeButton == "busRouting":
            data = be.walkPlusBusBackEnd(startInput, endInput)
            # print(data)
            if type(data) == tuple:
                buslist = data[0]
                noofstops = data[1]
                walkstmtstrt = data[2]
                walkstmtend = data[3]
                distance = data[4]
                time = data[5]
                endtime = str(round(current_time + float(time)))
                b = []
                for i in range(len(buslist)):
                    temp = ""
                    counter = 0
                    for n in range(len(buslist[i])):
                        if n == 0 or n == 1 or n == 4:
                            temp = temp + buslist[i][n]
                            if counter == 0:
                                temp = "Bus " + temp + " from "
                            if counter == 1:
                                temp = temp + " to "
                            counter += 1
                    b.append(temp)
                buslist = b
                print(buslist, "bus list")
                print("------------------------------------------")
                noofstops = "Total number of stops: " + str(noofstops)
                print(noofstops, "stops list")
                print("------------------------------------------")
                print(walkstmtend, "walk end")
                print("------------------------------------------")
                print(walkstmtstrt, "walk start")
                print("------------------------------------------")
                print(distance, "distance")
                print("------------------------------------------")
                print(time, "time")
                return render_template('user.html', startInput=startInput, endInput=endInput, Transportlist=buslist,
                                       stops=noofstops, walkEnd=walkstmtend, walkStart=walkstmtstrt, dist=distance,
                                       time=time, current_time=current_time, endtime=endtime)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
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
