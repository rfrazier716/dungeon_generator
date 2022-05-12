package main

import (
	"math"
	"testing"
)

func TestDistance(t *testing.T) {
    cases := []struct{
		p1 coord2D
		p2 coord2D
		expected float64 }{
			{NewCoord2D(0,0), NewCoord2D(0,1), 1.0},
			{NewCoord2D(1,1), NewCoord2D(2,2), math.Sqrt(2)},
			{NewCoord2D(2,2), NewCoord2D(1,1), math.Sqrt(2)},
		}


	// make a valid epsilon for floating point comparison
	epsilon := 0.00001 // 1e-5 should be fine
	for _, testCase := range cases {
		actual := distance(testCase.p1, testCase.p2)
		if math.Abs(actual-testCase.expected)>epsilon{
			t.Fatalf("Calculated Distance: %v exceeds expected: %v by more than allowable(%v)", actual, testCase.expected, epsilon)
		}
	}	
}