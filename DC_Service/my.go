package main

import (
	"fmt"
	"os"
    "os/exec"
)

func vars() (int , string){
    return 10000 , "is str"
}

func prints(){
    x, _ := vars()
    _, y := vars()
    fmt.Println("x=" , x ,"\n")
    fmt.Println("y=" , y)
}



func main() {
    var (
        d = "name"
    )
    cmd := exec.Command("python3" , "../common.py")
    cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
    
	e := cmd.Run()
	CheckError(e, d)
    prints()
}

func CheckError(e error, d string) {
    if e != nil {
		fmt.Println(e)
	}
    if d != "" {
        fmt.Println("为str")
    }
}

// func foo() (int, string) {
//     return 10, "Q1mi"
// }

// func mmymy() {
//     x, _ := foo()
//     _, y := foo()
//     fmt.Println("x=", x)
//     fmt.Println("y=", y)
// }

// func main() {
//     mmymy()
// }