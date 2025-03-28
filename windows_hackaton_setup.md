# Installation and setup instruction for preparation for Hackathon


1. install vscode
2. install python with option add to PATH!
3. install git
4. (install 7zip)


5. Open visual studio code (vscode)
- install extensions "python" and "gitgraph"
- open terminal
- confirm python is available (if not, install python with "add to path") 
- in terminal type pip install -e . (in code folder)


6. start coppeliasim, no further action required

7. Switch to vscode and open hackathon source folder (this folder)

8. in terminal, confirm that the current folder is this folder (the source folder)


9. execute: 
- `cp -R .\data\Original_YOLOv5s 'C:\\Users\\Loan\\AppData\\Local\\okinawa-ai-beach-robot\\beachbot\\Cache\\models\Original_YOLOv5s'`
- `cp -R ..\Models\beachbot_yolov5s_beach-cleaning-object-detection__v8-yolotrain__yolov5pytorch_320_finetune 'C:\\Users\\Loan\\AppData\\Local\\okinawa-ai-beach-robot\\beachbot\\Cache\\models\'`

10. Start beachbot app:
- change folder to app folder `cd app`
- execute `python .\beachbot_controller.py --sim`


