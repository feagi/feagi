

def utf_detection_logic(detection_list):
    # todo: Add a logic to account for cases were two top ranked items are too close
    # Identifies the detected UTF character with highest activity
    highest_ranked_item = '-'
    second_highest_ranked_item = '-'
    for item in detection_list:
        if highest_ranked_item == '-':
            highest_ranked_item = item
        else:
            if detection_list[item]['rank'] > detection_list[highest_ranked_item]['rank']:
                second_highest_ranked_item = highest_ranked_item
                highest_ranked_item = item
            elif second_highest_ranked_item == '-':
                second_highest_ranked_item = item
            else:
                if detection_list[item]['rank'] > detection_list[second_highest_ranked_item]['rank']:
                    second_highest_ranked_item = item

    # todo: export detection factor to genome not parameters
    detection_tolerance = 1.5
    if highest_ranked_item != '-' and second_highest_ranked_item == '-':
        print("Highest ranking number was chosen.")
        print("1st and 2nd highest ranked numbers are: ", highest_ranked_item, second_highest_ranked_item)
        return highest_ranked_item
    elif highest_ranked_item != '-' and second_highest_ranked_item != '-':
        if detection_list[highest_ranked_item]['rank'] / detection_list[second_highest_ranked_item]['rank'] > \
                detection_tolerance:
            print("Highest ranking number was chosen.")
            print("1st and 2nd highest ranked numbers are: ", highest_ranked_item, second_highest_ranked_item)
            return highest_ranked_item
        else:
            print(">>>> >>> >> >> >> >> > > Tolerance factor was not met!! !! !!")
            print("Highest and 2nd highest ranked numbers are: ", highest_ranked_item, second_highest_ranked_item)
            return '-'
    else:
        return '-'

    # list_length = len(detection_list)
    # if list_length == 1:
    #     for key in detection_list:
    #         return key
    # elif list_length >= 2 or list_length == 0:
    #     return '-'
    # else:
    #     temp = []
    #     counter = 0
    #     # print(">><<>><<>><<", detection_list)
    #     for key in detection_list:
    #         temp[counter] = (key, detection_list[key])
    #     if temp[0][1] > (3 * temp[1][1]):
    #         return temp[0][0]
    #     elif temp[1][1] > (3 * temp[0][1]):
    #         return temp[1][0]
    #     else:
    #         return '-'

    # Load copy of all MNIST training images into mnist_data in form of an iterator. Each object has image label + image

