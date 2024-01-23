import string
import random
from datetime import datetime


def id_gen(size=6, chars=string.ascii_uppercase + string.digits, signature=''):
    return (str(datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars) for
                                                                                  _ in range(size)))+signature
