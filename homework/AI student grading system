def calculate_average(marks):
    return sum(marks) / len(marks)

def grade_student(avg):
    if 85 < avg <= 100:
        return "Grade: A+"
    elif 75 < avg <= 85:
        return "Grade: A"
    elif 55 < avg <= 75:
        return "Grade: B+"
    elif 45 < avg <= 55:
        return "Grade: B"
    elif 35 <= avg <= 45:
        return "Grade: C"
    elif 25 <= avg < 35:
        return "Grade: C+"
    elif 0 <= avg < 25:
        return "Grade: W"
    else:
        return "Invalid Marks"

def main():
    subjects = ["Discrete Maths", "Engineering Maths", "DSA", "Intro to AIML", "Open elective"]
    marks = []
    for subject in subjects:
        mark = float(input(f"Enter your marks for {subject}: "))
        marks.append(mark)
    avg = calculate_average(marks)
    print(f"The average is: {avg:.2f}")
    print(grade_student(avg))

if __name__ == "__main__":
    main()
