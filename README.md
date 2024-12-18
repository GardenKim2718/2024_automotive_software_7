# Team7_Automotive_Software
Team repository for Automotive Software Class Final Project (2024 Fall)

Collaborators
----------------
Han Seokhui (Team Leader)

Kim Chungwon (Team Member)

Bagas Tegar B. (Team Member)

Rules for creating & maintaining branches
----------------
1. Create a new branch whenever making major changes within code or adding functions.
2. Set the branch name that can represent the new functionality. (eg: Kalman Filter, Time-to-Collision)
3. Request compare & pull after informing other team mates and it is confirmed that the change is stable.
4. The main branch should be remained as our 'stable' version of our project.

Coding Guidelines
----------------
1. Word spacing: Follow all the rules within PDF
2. Braces: combination of first and second style(the last one)
3. Indent: maintain current coding
4. Comments: Follow the examples within PDF, more will be added in further updates
5. Pseudo code: Still planning, might add within complicated algorithms
6. Variable Naming:
7. snake_case for all variables
8. PascalCase for functions
9. ~~For constants, we are using snake_case for now, but maybe updated into SCREAMING_SNAKE_CASE after discussion~~ --> maintain snake_case
10. Operators: follow the PDF examble
11. For the MISRA-C, we may abide it in later work, but for now we will not follow it as it requires massive changes including making new functions and code writing

Update
--------------
2024-12-06  Updated for coding guideline homework

2024-12-13  Code fix (now working)

2024-12-15  Introduced DBSCAN algorithm and Savitzky-Golay filter within Lane Detection

2024-12-16  Updated Obstacle Detection & Classification

2024-12-17  Updated path planning algorithm

2024-12-17  Updated lateral and longitudinal control algorithm

2024-12-18  Final Version (Submitted)

TO DO
--------------
~~Update the code to match Software Architecture Design~~

~~Implement path planning for merge~~

Branch Description(Describe what this branch is made for)
--------------
Testing merge and obstacle avoidance algorithm

Testing more control functionality

May be buggy, still WIP

Comments
--------------

To use manual input for testing:

-> src/app/autonomous_driving/launch/autonomous_driving.launch.xml

\<set: arg name="use_manual_inputs" default="true" \/\>
