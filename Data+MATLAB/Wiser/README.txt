##############################################################
################### NEW INSTRUCTIONS #########################
##############################################################

START WISER TRACKING SYSTEM
1. Plug in 4 wiser active antennas
2. Plug 1 active wiser antenna into comtroller computer's usb
3. Open wiser.exe
4. File -> Open -> Mechatronics_setup.xml
5. Configure -> Load Layout Picture -> Mechatronics_0.6_inches_per_pixel.png
6. Configure -> Load Identity File -> UI_ID_LIST_6_ANT.xml (if this fails, check the xml file for NUL characters and delete them)
7. Configure -> Enable USB Antenna (should already be active/checked from setup file)
8. Configure -> Discover Network (should find 5 antennas)
9. View -> Toggle Circles (may not be saved as active in setup)
10. Hover mouse pointer above an antenna and press 'r' on the keyboard to select
11. Configure -> calibrate antenna position (circles will show distance from each antenna)
12. calibrate all antenna positions by pressing 'r' with mouse pointer hovering above antenna in picture
13. Configure -> Stop Antenna Calibration
14. View -> Configure Tag Filtering -> set to 1
15. View -> Toggle Tag Filtering -> Make sure check is off?
16. Turn on active antenna for wiser to track
17. Active Tracking -> Find Active Tags (should find the one)
18. Active Tracking -> Run Active Tracking (circles will show determined loaction)
19. Active Tracking -> Stop Active Tracking (stop tracking in order to collect json data from API)
20. Wiser system is ready for data collection (and actively tracking)

START OPTITRACK TRACKING SYSTEM
1. Open the Motive application
2. File -> Open -> Latest SE423 .ttp project file
3. Place object in view and motive should see it as RigidBody 2
4. minimize Motive
5. double click SampleClient.bat on desktop to initialize tracking
6. minimize the cmd window that runs

START PYTHON DATA COLLECTION FOR BOTH SYSTEMS
1. double click NickW_wiser_data.bat (data collection begins from wiser and optitrack if above steps completed correctly)
2. If everything is good, data will begin printing in the cmd window
3. If something is wrong, a message will be written in the cmd window
4. Data collection creates/appends to the file NickW_posData.csv every ~10.5 minutes (50000 frames = 2000 data points)
5. NickW_posData.csv should be created wherever NickW_wiser_data.bat is exectued from

DATA ANALYSIS
1. Open the .csv file in MS Excel
2. Highlight column A, go to the data tab, and select "text to columns"
3. Delimited -> Next
4. Check delimiters: comma, space, other: ]
5. Uncheck "treat consecutive delimiters as one"
6. Finish
7. Delete empty columns and pay attention to x,y,z ordering (wiser_x and wiser_y currently switched)
8. Analyze data

RUNNING THE ROBOT CAR
1. Insert and turn on robot battery
2. Open cmd in windows and change directory to where code is located
3. move .bin file to robot car (> pscp Wiset_Test.bin root@192.168.1.72:)
4. start 2 putty sessions, one for optitrack and one for reset/load (> putty root@192.168.1.72)
5. start optitrack receiving in one putty session (> ./BasicOptitrackComm)
6. reset and load in other putty session (> ./DSP_Reset), (> ./DSP_Load Wiser_Test.bin)



##############################################################
################### OLD INSTRUCTIONS #########################
##############################################################

wiser virtual environment created using virtualenv 16.0.0
to create a new virtual environment (> virtualenv my_project)
#creates a folder named my_project that is isolated from the global python environment
type (> activate my_project) to use the new env then proceed as per usual

run cmd
change directory to C:\Users\dan5\wiser\Scripts (> cd wiser\Scripts)
> activate
(wiser) virtual environment should now be active
> python wiser_data.py
begins data collection from the wiser system
> ctrl-c 
program is terminated and data is saved in directory as posData.csv
> posData.csv
opens data file in MS excel
select column A
data ribbon -> text to columns
delimited -> next -> comma, space, other } -> finish
xyz position data is now in three separate columns 


double click NickW_wiser_data.bat in NickW_Wiser folder
>>> file runs and collects data until interrupted
ctrl-c to interrupt, but select n to stay in batch file and then press any key to exit
data is saved as posData.csv in NickW_Wiser folder