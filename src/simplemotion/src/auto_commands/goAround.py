# Arjun Viswanathan
# 3/9/23
# Go around a person who is standing right in front of you

# Pseudocode:
'''
Keep going forward 
If detecting someone in front of you, decide whether to go left or right around them
    Store direction picked initially

Use camera to detect if obstacle is in FOV or not
    If in FOV, keep turning
    If not, come back to normal

Actions:
    1. With decided direction, keep turning base while moving forward until obstacle no longer a threat
        Turn by some small value
            If LEFT, rotate by negative value
            If RIGHT, rotate by positive value
        Add up the values turned by for a final turn value
            Will be >0 or <0 depending on direction

    2. After out of threat, need to turn back to straight course
        Take value that was used to rotate slowly
            If <0, rotate by positive value and add
            If >0, rotate by negative value and subtract
        Once total value is 0, can stop
        At the end, will be facing front again

    3. Perform Actions 1 and 2 
'''