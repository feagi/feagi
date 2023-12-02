import datetime
import string
import random
import xxhash


def shuffle_set_contents(input_set):
    """
    Shuffles the contents of a set and returns them as a list.

    Parameters:
    input_set (set): The set to shuffle.

    Returns:
    list: A list with shuffled contents of the original set.
    """
    shuffled_list = list(input_set)
    random.shuffle(shuffled_list)
    return shuffled_list


def generate_serial_number_set(base, length, count):
    """
    Generates a set of unique serial numbers.

    Parameters:
    base (str): The base part of the serial number.
    length (int): The length of the unique part of the serial number.
    count (int): The number of serial numbers to generate.

    Returns:
    set: A set of unique serial numbers.
    """
    unique_set = set()
    while len(unique_set) < count:
        # Generate a random unique part
        unique_part = ''.join(random.choices(string.ascii_uppercase + string.digits, k=length)) + '_N'
        # Combine with the base part
        new_element = base + unique_part
        unique_set.add(new_element)

    return unique_set


def generate_unique_xxhash_cache(serial_numbers_set):
    combined_hash = 0
    hash_cache = {}

    for serial_number in serial_numbers_set:
        if serial_number not in hash_cache:
            # Compute and cache the hash for new serial numbers
            hash_cache[serial_number] = xxhash.xxh32(serial_number).intdigest()

        # Retrieve the hash from the cache
        individual_hash = hash_cache[serial_number]

        # Combine hashes using XOR
        combined_hash ^= individual_hash

    return hex(combined_hash)


base_part = '___pwr_231201221449362926_'
count = 1000000
serial_numbers = generate_serial_number_set(base_part, 5, count)


print("\nxxhash_w_cache")
for i in range(5):
    start = datetime.datetime.now()
    identifier = generate_unique_xxhash_cache(shuffle_set_contents(serial_numbers))
    duration = datetime.datetime.now() - start
    print(f"Unique Identifier: {identifier}", duration, duration / count, count)
