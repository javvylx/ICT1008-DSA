from flask import Flask, render_template
from flask import request
import backend as be
import datetime

app = Flask(__name__)

currentDT = datetime.datetime.now()
current_time = int(currentDT.strftime("%H%M"))


@app.route("/", methods=["GET", "POST"])
def plot():
    # POST The request from the web form
    if request.method == "POST":
        # Capture user input using POST
        capture_frontend_req = request.form.to_dict()
        startInput = capture_frontend_req['start_holder']
        endInput = capture_frontend_req['end_holder']
        typeButton = capture_frontend_req['buttonType']
        # If the radio button is selected for walk routing
        if typeButton == "walkRouting":
            # Runs walking function in Backend.py
            data = be.walkingBackEnd(startInput, endInput)
            # If the data type is a array, extract values of dist n time
            # Else just return the normal value of data value
            if type(data) == list:
                dist = data[0]
                time = data[1]
                endtime = str(round(current_time + float(time)))
                return render_template('user.html', startInput=startInput, endInput=endInput, dist=dist, time=time,
                                       current_time=current_time, endtime=endtime)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
            # If the radio button is selected for drive routing
        elif typeButton == "driveRouting":
            # Runs driving function in Backend.py
            data = be.drivingBackEnd(startInput, endInput)
            # print(data)
            if type(data) == list:
                distDrive = data[0]
                timeDrive = data[1]
                endtime = str(round(current_time + float(timeDrive)))
                return render_template('user.html', startInput=startInput, endInput=endInput, dist=distDrive,
                                       time=timeDrive, current_time=current_time, endtime=endtime)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
        elif typeButton == "busRouting":
            # Runs bus + walk function in Backend.py
            data = be.walkPlusBusBackEnd(startInput, endInput)
            # If the data type is a array, extract values of bus list, number of stops, walk start, end, dist, time, endtime
            # Else just return the normal value of data value
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
                return render_template('user.html', startInput=startInput, endInput=endInput, Transportlist=buslist,
                                       stops=noofstops, walkEnd=walkstmtend, walkStart=walkstmtstrt, dist=distance,
                                       time=time, current_time=current_time, endtime=endtime)
            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)

        elif typeButton == "lrtRouting":
            data = be.lrtBackEnd(startInput, endInput)
            # Runs LRT + Walking function in Backend.py once button is selected
            if type(data) == list:
                dist = data[0]
                time = data[1]
                endtime = str(round(current_time + float(time)))
                if len(data) == 4:
                    startlrt = "\nWalk from " + startInput + " to " + data[2] + " LRT station"
                    lrttrav = "Take the LRT from " + data[2] + " station to " + data[3] + " station"
                    endlrt = "Walk from " + data[3] + " LRT station " + " to " + endInput
                    directions = [startlrt, lrttrav, endlrt]
                    return render_template('user.html', startInput=startInput, endInput=endInput,
                                           TransportList=directions, stops=endlrt, walkEnd=lrttrav, walkStart=startlrt,
                                           dist=dist, time=time,
                                           current_time=current_time, endtime=endtime)
                elif len(data) == 5:
                    startlrt = "\nWalk from " + startInput + " to " + data[2] + " LRT station"
                    lrttrav = "Take the LRT from " + data[2] + " station to " + data[
                        3] + " station. \nThen take another LRT from " + data[3] + " station to " + data[
                                  4] + " station."
                    endlrt = "Walk from " + data[4] + " LRT station " + " to " + endInput
                    directions = [startlrt, lrttrav, endlrt]
                    return render_template('user.html', startInput=startInput, endInput=endInput,
                                           TransportList=directions, stops=endlrt, walkEnd=lrttrav, walkStart=startlrt,
                                           dist=dist, time=time,
                                           current_time=current_time, endtime=endtime)

            elif type(data) == str:
                return render_template('user.html', startInput=startInput, endInput=endInput, data=data)
    # Returns user template on run
    return render_template('user.html')


# Values are passed down from flask_control_start.py to user.html as a returned value and called in user.html as
#                     <b>Start Location:</b> <br>{{startInput}}
#                     <br> <b>End Location:</b><br> {{endInput}}
#                     <br> <b>Distance (km):</b><br>{{dist}}
#                     <br> <b>Time to reach (mins):</b> <br>{{time}}<br>
#                     <br> <b>Start Time (12-hour):</b><br>{{current_time}}<br>
#                     <br> <b>Estimated Arrival Time (12-hour):</b><br>{{endtime}}<br>

if __name__ == '__main__':
    app.run(debug=True)
