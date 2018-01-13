# NEverything

NEverything library lets you create **everything** you want, just straigt from the code.

## Introduction

This is really great library. And it contains GitHub logo, downloaded from... [GitHub](https://github.com/logos), what makes it event better:

![GitHub Logo](/doc/GitHub_Logo.png)

## Examples

If we do not want to use simple `NEverything.CreateAllWhatIWant().Run()`, we can be more precise. For example, in C#:

```csharp
var moneyGenerator = NEverything.CreateWhatIWant(_ => CreatesMoney());
var money = moneyGnerator.Run();
```