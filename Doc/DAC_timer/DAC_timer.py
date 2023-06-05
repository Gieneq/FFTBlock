f_clk = 64e6
f_target_list = [20, 30e3]
n_samples_list = [16,32,64,128,256,512,1024,2048]


def calculate_divider(f_clk, f_target, n_samples):
    return f_clk / (f_target * n_samples)
    

print(f"Source clock {f_clk/1e6} MHz")
print(f"{'n_smpls':<10}{'f_tar[Hz]':<14}{'div':<12}{'t[us]':<12}")
for n_samples in n_samples_list:
    for f_target in f_target_list:
        divider = calculate_divider(f_clk, f_target, n_samples)
        interval = 1/((f_clk/1e6)/divider)
        print(f"{n_samples:<10}{f_target:<14}{round(divider,2):<12}{round(interval,3):<12}")
        