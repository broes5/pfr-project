
print("hello")
INSTRUCTIONS="instructions.txt"

# Open instructions.txt and iterate over the lines.
with open(INSTRUCTIONS, 'r') as file:
    for line in file:
        print(line)
