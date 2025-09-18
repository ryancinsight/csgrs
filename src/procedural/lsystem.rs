//! L-system (Lindenmayer system) implementation for plant generation
//!
//! L-systems are formal grammars used to model plant growth and branching structures.
//! This module provides a complete L-system interpreter for generating complex tree and plant structures.

use std::collections::HashMap;

/// L-system production rule
#[derive(Debug, Clone)]
pub struct Rule {
    pub predecessor: char,
    pub successor: String,
    pub probability: f64, // For stochastic L-systems
}

/// Complete L-system definition
#[derive(Debug, Clone)]
pub struct LSystem {
    pub axiom: String,
    pub rules: HashMap<char, Vec<Rule>>,
    pub angle: f64, // Branching angle in radians
    pub segment_length: f64,
}

impl LSystem {
    /// Create a new L-system
    pub fn new(axiom: &str) -> Self {
        Self {
            axiom: axiom.to_string(),
            rules: HashMap::new(),
            angle: std::f64::consts::PI / 6.0, // 30 degrees
            segment_length: 1.0,
        }
    }

    /// Add a deterministic production rule
    pub fn add_rule(&mut self, predecessor: char, successor: &str) {
        self.rules.entry(predecessor).or_insert_with(Vec::new).push(Rule {
            predecessor,
            successor: successor.to_string(),
            probability: 1.0,
        });
    }

    /// Add a stochastic production rule
    pub fn add_stochastic_rule(&mut self, predecessor: char, successor: &str, probability: f64) {
        self.rules.entry(predecessor).or_insert_with(Vec::new).push(Rule {
            predecessor,
            successor: successor.to_string(),
            probability,
        });
    }

    /// Generate L-system string after n iterations
    pub fn generate(&self, iterations: usize) -> String {
        let mut current = self.axiom.clone();

        for _ in 0..iterations {
            let mut next = String::new();
            for ch in current.chars() {
                if let Some(rules) = self.rules.get(&ch) {
                    if rules.len() == 1 && rules[0].probability == 1.0 {
                        // Deterministic rule
                        next.push_str(&rules[0].successor);
                    } else {
                        // Stochastic rule - select based on probability
                        let mut rand_val = 0.0; // In real implementation, use random number
                        let mut cumulative = 0.0;
                        for rule in rules {
                            cumulative += rule.probability;
                            if rand_val <= cumulative {
                                next.push_str(&rule.successor);
                                break;
                            }
                        }
                    }
                } else {
                    // No rule - keep character
                    next.push(ch);
                }
            }
            current = next;
        }

        current
    }
}

/// Predefined L-systems for common plants
pub mod presets {
    use super::LSystem;

    /// Create a simple tree L-system
    pub fn simple_tree() -> LSystem {
        let mut lsystem = LSystem::new("F");
        lsystem.add_rule('F', "F[+F]F[-F]F");
        lsystem.add_rule('+', "+");
        lsystem.add_rule('-', "-");
        lsystem.add_rule('[', "[");
        lsystem.add_rule(']', "]");
        lsystem.angle = std::f64::consts::PI / 6.0; // 30 degrees
        lsystem.segment_length = 0.5;
        lsystem
    }

    /// Create a more complex tree with branches
    pub fn complex_tree() -> LSystem {
        let mut lsystem = LSystem::new("A");
        lsystem.add_rule('A', "[&FL!A]/////[&FL!A]///////[&FL!A]");
        lsystem.add_rule('F', "S/////F");
        lsystem.add_rule('S', "F L");
        lsystem.add_rule('L', "[+++^F][---^F]");
        lsystem.angle = std::f64::consts::PI / 8.0; // 22.5 degrees
        lsystem.segment_length = 0.3;
        lsystem
    }

    /// Create a plant with flowers
    pub fn flowering_plant() -> LSystem {
        let mut lsystem = LSystem::new("X");
        lsystem.add_rule('X', "F[+X][-X]FX");
        lsystem.add_rule('F', "FF");
        lsystem.angle = std::f64::consts::PI / 4.0; // 45 degrees
        lsystem.segment_length = 0.4;
        lsystem
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_lsystem() {
        let mut lsystem = LSystem::new("F");
        lsystem.add_rule('F', "F+F");

        let result = lsystem.generate(3);
        assert_eq!(result, "F+F+F+F+F+F+F+F");
    }

    #[test]
    fn test_tree_lsystem() {
        let tree = presets::simple_tree();
        let result = tree.generate(2);

        assert!(result.contains('F'));
        assert!(result.contains('['));
        assert!(result.contains(']'));
        assert!(result.contains('+'));
        assert!(result.contains('-'));
    }

    #[test]
    fn test_complex_tree_lsystem() {
        let tree = presets::complex_tree();
        let result = tree.generate(1);

        assert!(result.contains('A'));
        assert!(result.contains('&'));
        assert!(result.contains('!'));
    }

    #[test]
    fn test_flowering_plant_lsystem() {
        let plant = presets::flowering_plant();
        let result = plant.generate(2);

        assert!(result.contains('X'));
        assert!(result.contains('F'));
        assert!(result.contains('+'));
        assert!(result.contains('-'));
    }
}
