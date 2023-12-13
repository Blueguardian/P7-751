# Designed by group 7-751, Aalborg University, 2023 Fall-semester.
# For purpose of project, concerning tethered drone control.
# All rights reserved, can be copied under license.

import lxml.etree, lxml.objectify
import numpy as np
from io import StringIO

class XMLhandler():
    """
    XMLhandler class, for parsing and processing XML data in order to send it through sockets. Currently operated by
    utilising either process_xml and parse_xml when either is required. Assumes that all data input for parsing has been
    processed by the class.
    Dependencies:
        lxml.etree, lxml.objectify, numpy and io.StringIO
    Compatibility:
        All
    Further development:
        Ability to handle more types and classes
        More robust processing and parsing algorithms
        Cleanup and optimization
    """

    # Fixed class attributes and subclasses
    __BUILDER = lxml.etree
    __COUNTER = lxml.objectify

    @classmethod
    def __procxml_3Dmatrix(cls, data, _type, origin):
        """
        Classmethod for processing 3Dmatrix data into an XML format (Not optimized)
        :param data: Data to be processed
        :param _type: Type of data
        :return: Data in XML-bytestring format
        """
        ROOT = cls.__BUILDER.Element("Matrix3D", type=f"{_type}", origin=f"{origin}")
        ROWS = []
        INDEX = []
        for i in range(0, data.shape[0]):
            ROWS.append( cls.__BUILDER.SubElement(ROOT, "ROW", num=f"i"))
            for j in range(0, data.shape[1]):
                INDEX.append(cls.__BUILDER.SubElement(ROWS[i], f"INDEX", num=f"{j + 1}"))
                for p in range(0, data.shape[2]):
                    cls.__BUILDER.SubElement(INDEX[j], f"X{p + 1}").text = str(data[i, j, p])
        return ROOT

    @classmethod
    def __procxml_2Dmatrix(cls, data, _type, origin):
        """
        Classmethod for processing 2Dmatrix data into an XML format (Not optimized)
        :param data: Data to be processed
        :param _type: Type of data
        :return: Data in XML-bytestring format
        """
        ROOT = cls.__BUILDER.Element("Matrix", type=f"{_type}", origin=f"{origin}")
        ROWS = []
        for i in range(0, data.shape[0]):
            ROWS.append(cls.__BUILDER.SubElement(ROOT, "ROW", num=f"{i}"))
            for j in range(0, data.shape[1]):
                cls.__BUILDER.SubElement(ROWS[i], "INDEX", num=f"{j + 1}").text = str(data[i, j])
        return ROOT

    @classmethod
    def __procxml_transform(cls, data, _type, origin):
        """
        Classmethod for processing transformation matrix data into an XML format (Not optimized)
        :param data: Data to be processed
        :param _type: Type of data
        :return: Data in XML-bytestring format
        """
        ROOT = cls.__BUILDER.Element("Transform", type=f"{_type}", origin=f"{origin}")
        ROOT.text = _type
        ROW1 = cls.__BUILDER.SubElement(ROOT, "Row", num="1")
        ROW2 = cls.__BUILDER.SubElement(ROOT, "Row", num="2")
        ROW3 = cls.__BUILDER.SubElement(ROOT, "Row", num="3")
        ROW4 = cls.__BUILDER.SubElement(ROOT, "Row", num="4")
        for i in range(0, data.shape[0]):
            for j in range(0, data.shape[1]):
                cls.__BUILDER.SubElement(eval("ROW" + str(f"{i + 1}")), "INDEX", num=f"{j + 1}").text = str(data[i, j])
        return ROOT

    @classmethod
    def __procxml_vector(cls, data, _type, origin):
        """
        Classmethod for processing vectors data into an XML format (Not optimized)
        :param data: Data to be processed
        :param _type: Type of data
        :return: Data in XML-bytestring format
        """
        ROOT = cls.__BUILDER.Element("Vector", type=f"{_type}", origin=f"{origin}")
        ROOT.text = _type
        for i in range(0, len(data)):
            cls.__BUILDER.SubElement(ROOT, "INDEX", num=f"{i + 1}").text = str(data[i])
        return ROOT

    @classmethod
    def __procxml_value(cls, data, _type, origin):
        """
        Classmethod for processing single value data into an XML format (Not optimized)
        :param data: Data to be processed
        :param _type: Type of data
        :return: Data in XML-bytestring format
        """
        ROOT = cls.__BUILDER.Element("Value", type=f"{_type}", origin=f"{origin}")
        ROOT.text = _type
        cls.__BUILDER.SubElement(ROOT, "VALUE").text = str(data)
        return ROOT

    @classmethod
    def __np_3Dmatrix(cls, data, __type):
        """
        Classmethod for converting 3D matrix data into a numpy array (not tested or optimized)
        :param __type: Type of data
        :param data: data to be processed
        :return: Data in numpy array format
        """
        matrix = np.zeros(data.shape, dtype=eval(__type))
        for i in range(0, matrix.shape[0]):
            for j in range(0, matrix.shape[1]):
                for p in range(0, matrix.shape[2]):
                    matrix[i, j, p] = data[i, j, p]
        return matrix

    @classmethod
    def __np_2Dmatrix(cls, data, __type):
        """
        Classmethod for converting 2D matrix data into a numpy array (not tested or optimized)
        :param __type: Type of data
        :return: Data in numpy array format
        """
        matrix = np.zeros(data.shape, dtype=eval(__type))
        for i in range(0, matrix.shape[0]):
            for j in range(0, matrix.shape[1]):
                matrix[i, j] = data[i, j]
        return matrix

    @classmethod
    def __np_transform(cls, data, __type):
        """
        Classmethod for converting transformation matrix data into a numpy array (not tested or optimized)
        :param __type: Type of data
        :return: Data in numpy array format
        """
        transform = np.zeros((4, 4), dtype=eval(__type))
        for i in range(0, transform.shape[0]):
            for j in range(0, transform.shape[1]):
                transform[i, j] = data[i, j]
        return transform

    @classmethod
    def __np_vector(cls, data, __type):
        """
        Classmethod for converting vector into a numpy array (not tested or optimized)
        :param __type: Type of data
        :return: Data in numpy array format
        """
        vector = np.array(data, dtype=eval(__type))
        return vector

    @classmethod
    def __np_value(cls, value, __type):
        """
        Classmethod for converting single value data into a numpy array (not tested or optimized)
        :param __type: Type of data
        :return: Data in numpy array format
        """
        value = np.array(value, dtype=eval(__type))
        return value

    @classmethod
    def __det_type(cls, data):
        """
        Classmethod for determining type of data and output it in relevant format for xml processing
        :param data: data to be processed
        :return: type of data or element type of list, tuple or numpy array
        """
        __type = ''
        if isinstance(data, list) or isinstance(data, tuple):
            __type = np.array(data).dtype.name
            return (type(data), f"np.{__type}")
        elif isinstance(data, np.ndarray):
            __type = data.dtype.name
            return (np.ndarray, f"np.{__type}")

        if isinstance(data, complex):
            __type = eval(f"np.{type(data).__name__}"+"124")
            return __type
        elif isinstance(data, int) or isinstance(data, float):
            __type = eval(f"np.{type(data).__name__}"+"32")
            return __type
        elif isinstance(data, str):
            __type = eval(f"np.{type(data).__name__}"+"_")
            return __type
        elif isinstance(data, bool):
            __type = eval(f"np.{type(data).__name__}"+"_")
            return __type
        else:
            raise Exception("Non-compatible data type for object")

    def process_xml(self, data, origin="None"):
        """
        Instance method for processing data into xml format, utilises classmethods for processing
        :param data: Data to be processed
        :param origin: Origin of data
        :return: Data in xml bytestring format
        """
        __type = self.__det_type(data)
        xml_data = None
        if isinstance(__type, tuple):
            __type = __type[1]
        data_dim = np.array(data, dtype=eval(__type)).ndim
        if data_dim == 3:
            xml_data = self.__procxml_3Dmatrix(self.__np_3Dmatrix(data, __type), __type, origin)
        elif data_dim == 2 and data.shape[0] == 4 and data.shape[1] == 4:
            xml_data = self.__procxml_transform(self.__np_transform(data, __type), __type, origin)
        elif data_dim == 2 and data.shape[0] != 4 and data.shape[1] != 4:
            xml_data = self.__procxml_2Dmatrix(self.__np_2Dmatrix(data, __type), __type, origin)
        elif data_dim == 1:
            xml_data = self.__procxml_vector(self.__np_vector(data, __type), __type, origin)
        elif data_dim == 0:
            xml_data = self.__procxml_value(self.__np_value(data, __type), __type, origin)
        else:
            raise Exception("Unknown dimensions")
        return lxml.etree.tostring(xml_data)

    @classmethod
    def __parsexml_3Dmatrix(cls, tree, data):
        """
        Classmethod for parsing a xml bytestring of a 3D matrix into a numpy array
        :param tree: The element tree object containing the data in xml format
        :param data: The data to be parsed
        :return: a numpy array corresponding to the data
        """
        ROOT = tree.getroot()
        __type = eval(ROOT.get('type'))

        counter = cls.__COUNTER.parse(StringIO(data))
        root = counter.getroot()
        x = root.countchildren()
        y = root.ROW.countchildren()
        z = root.ROW.INDEX.countchildren()
        matrix3d = np.zeros((x, y, z), dtype=__type)
        origin = ROOT.get("origin")

        i = j = k = 0
        for row in ROOT:
            for element in row:
                for index in element:
                    matrix3d[i, j, k] = index.text
                    k += 1
                j += 1
                k = 0
            i += 1
            j = 0
        return origin, matrix3d

    @classmethod
    def __parsexml_2Dmatrix(cls, tree, data):
        """
        Classmethod for parsing a xml bytestring of a 2D matrix into a numpy array
        :param tree: The element tree object containing the data in xml format
        :param data: The data to be parsed
        :return: a numpy array corresponding to the data
        """
        ROOT = tree.getroot()
        __type = eval(ROOT.get('type'))

        counter = cls.__COUNTER.parse(StringIO(data))
        root = counter.getroot()
        x = root.countchildren()
        y = root.ROW.countchildren()
        matrix2d = np.zeros(shape=(x, y), dtype=__type)
        origin = ROOT.get("origin")

        i = j = 0
        for row in ROOT:
            for element in row:
                matrix2d[i, j] = element.text
                j += 1
            i += 1
            j = 0
        return origin, matrix2d

    @classmethod
    def __parsexml_transform(cls, tree):
        """
        Classmethod for parsing a xml bytestring of a transformation matrix into a numpy array
        :param tree: The element tree object containing the data in xml format
        :return: a numpy array corresponding to the data
        """
        ROOT = tree.getroot()
        __type = eval(ROOT.get('type'))
        transform = np.zeros((4, 4), dtype=__type)
        origin = ROOT.get("origin")

        i = j = 0
        for row in ROOT:
            for element in row:
                transform[i, j] = element.text
                j += 1
            i += 1
            j = 0
        return origin, transform

    @classmethod
    def __parsexml_vector(cls, tree):
        """
        Classmethod for parsing a xml bytestring of a vector into a numpy array
        :param tree: The element tree object containing the data in xml format
        :return: a numpy array corresponding to the data
        """
        ROOT = tree.getroot()
        __type = eval(ROOT.get('type'))
        vector = []
        origin = ROOT.get("origin")

        for element in ROOT:
            vector.append(element.text)
        vector = np.array(vector, dtype=__type)
        return origin, vector

    @classmethod
    def __parsexml_value(cls, tree):
        """
        Classmethod for parsing a xml bytestring of a single value into a numpy array
        :param tree: The element tree object containing the data in xml format
        :return: a numpy array corresponding to the data
        """
        ROOT = tree.getroot()
        __type = eval(ROOT.get('type'))
        value = None
        origin = ROOT.get("origin")

        for element in ROOT:
            value = element.text
        value = np.array(value, dtype=__type)
        return origin, value

    def parse_xml(self, data):
        """
        Instance method, used in determining what type of data was received and how to process it to obtain
        relevant information. Parses the information in regard to the obtained information and returns the data in
        numpy array format.
        :param data: Data to be parsed
        :return: numpy array format of data
        """
        tree = self.__BUILDER.parse(StringIO(data))
        ROOT = tree.getroot()
        tag = ROOT.tag
        data_parsed = None
        origin = None

        if tag == 'Value':
            origin, data_parsed = self.__parsexml_value(tree)
        elif tag == 'Vector':
            origin, data_parsed = self.__parsexml_vector(tree)
        elif tag == 'Transform':
            origin, data_parsed = self.__parsexml_transform(tree)
        elif tag == 'Matrix':
            origin, data_parsed = self.__parsexml_2Dmatrix(tree, data)
        elif tag == 'Matrix3D':
            origin, data_parsed = self.__parsexml_3Dmatrix(tree, data)
        else:
            raise Exception("Unknown data")
        return origin, data_parsed
