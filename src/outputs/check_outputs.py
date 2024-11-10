file1 = 'chesapeake-td_38_0_8_par_set_output.txt'
file2 = 'chesapeake-td_38_0_output.txt'

# file1 = 'luxembourg-td_1001_0_8_par_set_output.txt'
# file2 = 'luxembourg-td_1001_0_output.txt'

with open(file1, 'r') as f1, open(file2, 'r') as f2:
    file1_lines = f1.readlines()
    file2_lines = f2.readlines()

if len(file1_lines) != len(file2_lines):
    print("Files do not have the same number of lines.")
else:
    for line1, line2 in zip(file1_lines, file2_lines):
        if line1 != line2:
            print("Mismatch found:")
            print(f"File1: {line1.strip()}")
            print(f"File2: {line2.strip()}")
            break
    else:
        print("All lines are equal.")
