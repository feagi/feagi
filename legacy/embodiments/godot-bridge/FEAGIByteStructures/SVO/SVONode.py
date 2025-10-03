# No need to import anything lol

class SVONode:
    """
    Individual SVO (non-leaf) Node
    """

    def __init__(self):
        self.child_bitmask: int = 0  # 8-bit mask (which of 8 children exist)
        self.children: list[SVONode or None] = [None] * 8
