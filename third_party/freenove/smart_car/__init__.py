from pyfirmata import util, Arduino
import time
import numpy as np

#board = Arduino('/dev/ttyACM0')
#it = util.Iterator(board)
#it.start()


class Seven_segment_display:
    def seven_segment_display(self, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7, pin_8, seconds):
        """
        Parameters
        ----------
        self
        pin_2, pin_3, pin_4, pin_5, pin_6, pin_7. and pin_8 are the pin where it can light the one segment each.
        Each pin with number should be on the same number as digital on the board.

        seconds is the second of how long will it stay on. It'll be used with time.sleep(seconds)
        -------
        """
        if (pin_2 < 2 or pin_2 > 0):
            board.digital[2].write(pin_2)
        else:
            "Please type 0 or 1 only"
        if (pin_3 < 2 or pin_3 > 0):
            board.digital[3].write(pin_3)
        else:
            "Please type 0 or 1 only"
        if (pin_4 < 2 or pin_4 > 0):
            board.digital[4].write(pin_4)
        else:
            "Please type 0 or 1 only"
        if (pin_5 < 2 or pin_5 > 0):
            board.digital[5].write(pin_5)
        else:
            "Please type 0 or 1 only"
        if (pin_6 < 2 or pin_6 > 0):
            board.digital[6].write(pin_6)
        else:
            "Please type 0 or 1 only"
        if (pin_7 < 2 or pin_7 > 0):
            board.digital[7].write(pin_7)
        else:
            "Please type 0 or 1 only"
        if (pin_8 < 2 or pin_8 > 0):
            board.digital[pin_8]
        else:
            "Please type 0 or 1 only"
        time.sleep(seconds)  ##This is in second. If you need to put 1 ms, put 0.001

    def destroy_one(self):
        """
        Once you want to shut all light off, use destroy(). It will be an instant action.
        """
        destroy = Seven_segment_display()
        destroy.seven_segment_display(0, 0, 0, 0, 0, 0, 0, 0)

    def four_seven_displays(self):
        """
        Parameters
        ----------
        Here is the map of D and pin on the arduino.
        pin_A = digital 2
        pin_B = digital 3
        pin_C = digital 4
        pin_D = digital 5
        pin_E = digital 6
        pin_F = digital 7
        pin_G = digital 8
        D1 = digital 9
        D2 = digital 10
        D3 = digital 11
        D4 = digital 12
        seconds = seconds
        Returns
        -------
         [(1,0,1,1, 1, 1, 0, 1), (0,1,0,1, 0, 1, 1, 1), (1,0,0, 0, 0, 0, 0, 1), (1,0,1,0, 1, 1, 1, 1), 5)]
        #[(a1, b1, c1, d1, e1, f1, g1), (a2, b2, c2, d2, e2, f2, g2), (a3, b3, c3, d3, e3, f3, g3),
        # (a4, b4, c4, d4, e4, f4, g4), 5)]
        total=np.full((4,7), (a, b, c, d, e, f, g))
        new = total.size
        print(new)
        """

        # [(a1,b1,c1,d1, e1, f1, g1, dp1), (a2,b2,c2,d2, e2, f2, g2, dp2), (a3,b3,c3, d3, e3, f3, g3, dp3), (a4,b4,c4,d4, e4, f4, g4, dp4), time)
        # [(0,0,0,0, 0, 0, 1, 1), (0,1,1,0, 1, 1, 0, 1), (1,1,1, 0, 1, 1, 1, 1), (1,0,1,0, 0, 0, 0, 1)]
        # total=np.full((4,7), (a, b, c, d, e, f, g))

        data = [[0, 0, 0, 0, 0, 0, 1, 1], [0, 1, 1, 0, 1, 1, 0, 1], [1, 1, 1, 0, 1, 1, 1, 1], [1, 0, 1, 0, 0, 0, 0, 1]]
        data1 = np.array(data)
        # data[dp1] = [a1, b1, c1, d1, e1, f1, g1]
        # data[dp2] = [a2, b2, c2, d2, e2, f2, g2]
        # data[dp3] = [a3, b3, c3, d3, e3, f3, g3]
        # data[dp4] = [a4, b4, c4, d4, e4, f4, g4]
        # total=pin_info
        print(len(data1))  ##This counts the dimensions
        dimension = range(len(data1))
        print(data1)
        for index in dimension:  ##loop by the amount of dimension which will be used for dp

            # board.digital[(9 + index)].write(data1[] if pin_dp > 0 and pin_dp <= 1 else 0)  ## Starting at 9 due to Dp starts at 9 to 12
            """

            row, col = total.shape  # just to get the number of column
            print(col)  # counts the column
            for x in range(col):
                num = total[index, x]  # to obtain the value from the input in the function
            board.digital[x + 1].write(num if num > 0 and num <= 1 else 0)  # power per pin
            time.sleep(seconds)
    """

    def destroy_four(self):
        """
        THis is to turn all displays off instantly. It's highly recommended to use after you are done.
        """
        destroy = Seven_segment_display()
        destroy.four_seven_displays(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


if __name__ == '__main__':
    new = Seven_segment_display()
    new.four_seven_displays()
    #new.destroy_four()