import yaml


class Locator:
    """Locator to find position of a book in a library
    Initially implemented on the Imperial College Library,
    is based on a yaml file containing the first book of a shelve
    and the position of the shelve.
    """
    MAX_CODE_LENGTH = 9

    def __init__(self,shelveCatalogue):
        self.shelves = list()
        book_list = yaml.load(open(shelveCatalogue))
        for key, value in book_list.items():
            numerical_code = self.numericalCodeGenerator(key)
            self.shelves.append([numerical_code,
                                 [float(i) for i in value[0]],
                                 [float(i) for i in value[1]]])
        self.shelves.sort()


    def numericalCodeGenerator(self, key):
        """Takes code relating to a book and returns numerical format for ordering
        A code in the form of an integer is necessary to define what books come first
        and where they are located.
        Args:
            the code as returned from the lib server
        Returns:
             a numerical code to numerically order the books
             format of the numerical code is:
             9 digits containing as many numerical digits present on the original code
             and reported in the same format plus as many zeros as necessary to fill the
             code with 9 digits
             6 digits representing the conversion of the alphabetical triplete
        """
        keyClean = key.replace(".", "")  # delete dots from book code
        numbers, characters = keyClean.split(' ')
        compZeros = '0' * (self.MAX_CODE_LENGTH - len(numbers))
        unicodeCharacters = [str(ord(c)) for c in characters.upper()]
        numericalCode = int(numbers + compZeros + ''.join(unicodeCharacters))
        return numericalCode

    def returnLocation(self, bookCode):
        """Takes a book code and returns the position of the shelve
        that contains it
        """
        numericalBookCode = self.numericalCodeGenerator(bookCode)
        location, orientation = ([0,0,0], [0,0,0,1])
        for indexShelve in range(len(self.shelves)):
            if numericalBookCode < self.shelves[indexShelve][0]:
                location = self.shelves[indexShelve-1][1]
                orientation = self.shelves[indexShelve-1][2]
                break
        return location, orientation


if __name__ == "__main__":
    example = Locator("lib_ICL_5_floor.yaml")
    location, orientation = example.returnLocation("722.23 EDF")
    print(location)