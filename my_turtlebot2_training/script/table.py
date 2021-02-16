
# !/usr/bin/env python
# import xlsxwriter
import mysql.connector
from mysql.connector import errorcode


class TableGenerator:
    def __init__(self, table_name):
        try:
            self.cnx = mysql.connector.connect(
                host='127.0.0.1',
                user='root',
                password='1234567890',
                database='mydatabase'
            )
            self.cursor = self.cnx.cursor()
            self.cursor.execute('CREATE TABLE ' + table_name + ' (a_s VARCHAR(255), state VARCHAR(255))')
            print("TABLE " + table_name + " IS CREATED!")
        except mysql.connector.Error as err:
            if err.errno == errorcode.ER_ACCESS_DENIED_ERROR:
                print("Something is wrong with your user name or password")
            elif err.errno == errorcode.ER_BAD_DB_ERROR:
                print("Database does not exist")
            else:
                print(err)
        else:
            print("close")
            self.cnx.close()


#     def writ_table_data(table_name, value):
#         sql = "INSERT INTO " + table_name + " (name, address) VALUES (%s, %s)"
#         val = value
#         cursor.execute(sql, val)
#
#
#     def fetch_table_data(table_name):
#         try:
#             cnx = mysql.connector.connect(
#                 host='127.0.0.1',
#                 user='root',
#                 password='1234567890',
#                 database='mydatabase'
#             )
#             cursor = cnx.cursor()
#             # cursor.execute('CREATE TABLE ' + table_name + ' (a_s VARCHAR(255), state VARCHAR(255))')
#             cursor.execute('select * from ' + table_name)
#
#             header = [row[0] for row in cursor.description]
#
#             rows = cursor.fetchall()
#
#
#
#             # Closing connection
#             # cnx.close()
#             # print("done")
#             # return header, rows
#         except mysql.connector.Error as err:
#             if err.errno == errorcode.ER_ACCESS_DENIED_ERROR:
#                 print("Something is wrong with your user name or password")
#             elif err.errno == errorcode.ER_BAD_DB_ERROR:
#                 print("Database does not exist")
#             else:
#                 print(err)
#         else:
#             print("close")
#             cnx.close()
#             return header, rows
#
#
#     def export(table_name):
#         # Create an new Excel file and add a worksheet.
#         # workbook = xlsxwriter.Workbook(table_name + '.xlsx')
#         # worksheet = workbook.add_worksheet('MENU')
#         #
#         # # Create style for cells
#         # header_cell_format = workbook.add_format({'bold': True, 'border': True, 'bg_color': 'yellow'})
#         # body_cell_format = workbook.add_format({'border': True})
#
#         header, rows = fetch_table_data(table_name)
#         # fetch_table_data(table_name)
#         print(header)
#         print(rows)
#
#         # row_index = 0
#         # column_index = 0
#         #
#         # for column_name in header:
#         #     worksheet.write(row_index, column_index, column_name, header_cell_format)
#         #     column_index += 1
#         #
#         # row_index += 1
#         # for row in rows:
#         #     column_index = 0
#         #     for column in row:
#         #         worksheet.write(row_index, column_index, column, body_cell_format)
#         #         column_index += 1
#         #     row_index += 1
#         #
#         # print(str(row_index) + ' rows written successfully to ' + workbook.filename)
#         #
#         # # Closing workbook
#         # workbook.close()
#
#
# # Tables to be exported
# # export('TABLE1_EXP1')
# # export('TABLE2_EXP2')

if __name__ == '__main__':
    tbl = TableGenerator('test3')



