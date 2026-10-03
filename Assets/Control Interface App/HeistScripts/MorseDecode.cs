using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class MorseDecode : MonoBehaviour
{
    [SerializeField] private TMP_InputField morseInputField;
    [SerializeField] private TextMeshProUGUI decodedOutputText;
    [SerializeField] private bool decodeOnValueChange = false;

    private readonly Dictionary<string, string> morseToCharacter = new Dictionary<string, string>
    {
        {".-", "A"},
        {"-...", "B"},
        {"-.-.", "C"},
        {"-..", "D"},
        {".", "E"},
        {"..-.", "F"},
        {"--.", "G"},
        {"....", "H"},
        {"..", "I"},
        {".---", "J"},
        {"-.-", "K"},
        {".-..", "L"},
        {"--", "M"},
        {"-.", "N"},
        {"---", "O"},
        {".--.", "P"},
        {"--.-", "Q"},
        {".-.", "R"},
        {"...", "S"},
        {"-", "T"},
        {"..-", "U"},
        {"...-", "V"},
        {".--", "W"},
        {"-..-", "X"},
        {"-.--", "Y"},
        {"--..", "Z"},
        {".----", "1"},
        {"..---", "2"},
        {"...--", "3"},
        {"....-", "4"},
        {".....", "5"},
        {"-....", "6"},
        {"--...", "7"},
        {"---..", "8"},
        {"----.", "9"},
        {"-----", "0"},
        {".-.-.-", "."},
        {"--..--", ","},
        {"..--..", "?"},
        {".----.", "'"},
        {"-.-.--", "!"},
        {"-..-.", "/"},
        {"-....-", "-"},
        {".-..-.", "\""},
        {".--.-.", "@"}
    };

    private void Awake()
    {
        if (decodeOnValueChange && morseInputField != null)
        {
            morseInputField.onValueChanged.AddListener(_ => DecodeInputField());
        }
    }

    public void DecodeInputField()
    {
        if (morseInputField == null)
        {
            Debug.LogWarning("Morse input field is not assigned.");
            return;
        }

        string decoded = DecodeMorseString(morseInputField.text);

        if (decodedOutputText != null)
        {
            decodedOutputText.text = "Decoded text: " + decoded;
        }
    }

    public string DecodeMorseString(string morseInput)
    {
        if (string.IsNullOrWhiteSpace(morseInput))
        {
            return string.Empty;
        }

        List<string> decodedWords = new List<string>();

        string[] words = morseInput.Split(new[] { '/' }, StringSplitOptions.RemoveEmptyEntries);
        foreach (string word in words)
        {
            string[] symbols = word.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            string decodedWord = string.Empty;

            foreach (string symbol in symbols)
            {
                if (morseToCharacter.TryGetValue(symbol, out string decodedSymbol))
                {
                    decodedWord += decodedSymbol;
                }
                else
                {
                    decodedWord += "?";
                }
            }

            decodedWords.Add(decodedWord);
        }

        return string.Join(" ", decodedWords);
    }
}
