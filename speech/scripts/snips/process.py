import pandas as pd
import numpy as np
import re

pd.set_option('display.expand_frame_repr', False)
pd.set_option('max_colwidth', 800)

file_name =  'Sample-book-titles-and-authors-at-Central.xlsx'
sheet =  'Sheet1'
df = pd.read_excel(io=file_name, sheet_name=sheet, header=None)
df1 = df.replace(np.nan, '', regex=True)
print(df.head(5))  # print first 5 rows of the dataframe


titles= open("titles.yaml","wb")
#authors= open("authors.yaml","wb")


array = np.array(df1)


for row in array:
    
    title = row[0]
    if not title == "":
        if not re.search('\d+', title):
            if len(title.split(' ')) < 6:
                if title.startswith('#') or title.startswith('!') or title.startswith('*'):
                    title = title[1:]
                title = title.replace(':', '')
                title = title.replace('"', '')
                title = title.replace("'", '')
                title = title.replace("[", '')
                title = title.replace("]", '')
                title = title.strip('/ .')
                titles.write(('- ' + title + '\n').encode("utf-8"))
    
    
    """
    author = row[1]
    if not author == "":
        if not re.search('\d+', author):
            if author.startswith('#') or author.startswith('!') or author.startswith('*'):
                author = author[1:]
            author = author.replace('"', '')
            author = author.replace("'", '')
            author = author.replace("[", '')
            author = author.replace("]", '')
            author = author.replace(':', '')
            authors.write(('- ' + author + '\n').encode('utf-8'))
    """    
    
titles.close()
#authors.close()

