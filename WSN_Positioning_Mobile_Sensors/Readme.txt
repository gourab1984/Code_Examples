Program Abstract:

Patient call systems are used in many hospitals and old-age homes for additional patient safety. Patients might carry some kind of "remote buttons" for example on their wrists and/or there are alarm call buttons in their rooms, so that they can request aid quickly and easily in case of emergency.

In this project we implement this kind of system with the TUTWSN sensor network. The described system will be extended with staff respond devices, which function a little like ordinary paging devices: when an alarm occurs the system will forward the alarm to these staff devices. Mobile nodes and cell phones will be used together to simulate these devices.

The system implements the following functionalities:

1. The system will have two kinds of alarms: patient calls/alarms from the patient call devices, which are triggered with the push buttons, and patient condition alarms which trigger automatically when acceleration of the patient call device changes direction. This alarm states that the patient has fallen to ground.

2. When an alarm occurs the system must send a SMS (or email, if you do not want to use SMS) to a staff member nearest to the alarm source. The alarm message must contain at least patient's name/ID, location, timestamp and reason for the alarm (patient help call or a fallen person)

3. If the nearest staff member can't go and resolve the situation she must send a not-acknowledge information with her staff respond device (her mobile node). In this case the system will send the same alarm SMS to next staff member.

4. The system will automatically acknowledge and stop the alarm when staff respond device is located next to the patient call device.

5. The positioning is improved (more precise) by checking the Received Signal Strength Index (RSSI) values of the neighbor packets and averaging the position (e.g. 1234:3, 4321:2 and 5432:2, presented as Node_ID: RSSI. The position should be near node 1234 and on that direction while nodes 4321 and 5432 are also located).

Approximate guide for RSSI vs. Distance:

RSSI		Distance (m or xy coordinates)
3		<5
2		<10
1		<15
0		<50
