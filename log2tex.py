with open("output.txt", "r") as f:
    lines = f.readlines()

n = len(lines)
i = 0

st = {}

while i < n:
    scope = lines[i:i + 10]
    _, input_type, _, amp, _, period = scope[1].split()
    st[(float(input_type), float(amp), int(period))] = [float(line.split(':')[1].strip()) for line in scope[2:-1]]
    i += 10

# print(st)

with open("tex.txt", 'w') as f:
    for input_type in (1., 0.):
        amp_list = (5., 10., 15.) if input_type else (2., 6., 10.)
        for period in range(6, 11):
            f.write("{}s & ".format(period))
            for i, amplitude in enumerate(amp_list):
                f.write(" & ".join(str(x) for x in st[(input_type, amplitude / 100, period)]))
                if i != 2:
                    f.write(" & ")
            f.write("\\\\\n")
        f.write('\n')
