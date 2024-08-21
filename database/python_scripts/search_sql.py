import sqlite3

try:
    # 连接到 SQLite 数据库
    conn = sqlite3.connect('../delivery.db')
    # 连接到delivery.db数据库
    # conn = sqlite3.connect('../delivery.db')
    print("Opened database successfully")
    
    # 创建一个游标对象
    cursor = conn.cursor()
    
    # 查询数据
    # cursor.execute('SELECT * FROM express_info')
    # deliveries
    cursor.execute('SELECT * FROM deliveries')
    rows = cursor.fetchall()
    
    # 打印查询结果
    for row in rows:
        print(row)
except sqlite3.Error as e:
    print(f"An error occurred: {e}")
finally:
    # 关闭连接
    if conn:
        conn.close()
