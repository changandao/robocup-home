# Dependencies

* nltk
* aiml

To get all the dependencies:
```
make reqs
```

# Testing and training
## Intent Classification
For correct intent classification we need to have a correct set of training data. This is just the intent and sentence.
This will enable us to recognize the same intention of the operator even if we didn't recognize all the words correctly.

To test run:
```
roslaunch tum_alle_athome_speech_nlp speech_and_person.launch
```

And from the second terminal call the service with any question you think we may hear during the SPR challenge [See the rulebook.](https://latexonline.cc/compile?git=git%3A%2F%2Fgithub.com%2FRoboCupAtHome%2FRuleBook.git&target=Rulebook.tex&trackId=1499942385669)
```
rosservice call /tiago/speech/classify_intent "question: '[YOUR QUESTION]'"
```

If you can see correct intention, then cool, try one more. But if not, you can add your question to the list below (UNRECOGNIZED QUESTIONS)

# Running
## AIML (predefined questions)
```
rosrun tum_alle_athome_speech_nlp predefined.py
```

## Intent Classification
```
rosrun tum_alle_athome_speech_nlp intent.py
```

## NLP Core
```
rosrun tum_alle_athome_speech_nlp generate_answer.py
```

# UNRECOGNIZED QUESTIONS

### Question category: 1 [p]redefined, 2 [a]rena, 3 c[r]owd, 4 [o]bject

```
00X Question number.
"xxxx" Question generated from SPR command generator.
"xxxx" Answer predefined.
[x] Question category.
xxx_xxx True question intent classification.
The intent provided from our nodes is on screen.
```

```
001 "Which robot is used in the Open Platform League" "There is no standard defined for OPL" [p] predefined
```
answer: It was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_objects', 0.4103386809269162)

It was founded in 2013

```
002 "What is the colour of the bowl" "" [o] object_color
```

answer: We have 2017


('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
003 "How many fridge are in the living room" "" [a] arena_number
```

answer: answer: There are $answer fridge in the living room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 1.3359339190221542)

('How many fridge are in the living room', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('fridge', 'NN'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('living', 'NN'), ('room', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'fridge', 'properties': []})

```
004 "What objects are stored in the bedside" "" [o] arena_objects
```
answer: It was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.2681818181818182)

The first competition took place in 2013

```
005 "Tell me if the sitting person was a woman" "" [r] crowd_gender
```

answer: answer: Sorry, I cannot answer that question. I do not know what gender me is.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.2681818181818182)

The first competition took place in 2013

are('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 2.861038961038961)

('Tell me if the sitting person was a woman', 'crowd_gender')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('if', 'IN'), ('the', 'DT'), ('sitting', 'VBG'), ('person', 'NN'), ('was', 'VBD'), ('a', 'DT'), ('woman', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('me', 'PRP')

('Object: ', {'object': 'woman', 'properties': []})

```
006 "What is the name of your team" "We called ourselves Alle At Home" [p] predefined
```

answer: We called ourselves Alle At Home

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.5073934837092733)

('General', 'arena_objects', 0.1515151515151515)

We called ourselves Alle At Home

```
007 "How many girls are in the crowd" "" [r] crowd_number
```

answer: answer: There are  is girls in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.2734339190221542)

('How many girls are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('girls', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'girls', 'properties': []})

```
008 "In which room is the TV stand" "" [a] object_storage
```
answer: answer: There are $answer stand in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the TV stand', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('TV', 'NN'), ('stand', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'stand', 'properties': []})

```
009 "In which room is the cupboard" [a] object_storage
```

answer: answer: There are $answer cupboard in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.2770053475935828)

('In which room is the cupboard', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('cupboard', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'cupboard', 'properties': []})

```
010 "How many people in the crowd are standing or lying" "" [r] crowd_number
```
answer: answer: There are  is people in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 3.648433919022154)

('How many people in the crowd are standing or lying', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN'), ('are', 'VBP'), ('standing', 'VBG'), ('or', 'CC'), ('lying', 'VBG')])

There are $properties $person $object in $location.

('Object: ', {'object': 'people', 'properties': []})

```
011 "Where can I find a melon" "" [o] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I am from Barcelona

```
012 "In which room is the cabinet" "" [a] object_storage
```

answer: answer: There are $answer cabinet in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the cabinet', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('cabinet', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'cabinet', 'properties': []})

```
013 "Tell me how many people were wearing white" "" [r] crowd_number
```

answer: answer: There are me is people in .

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.09523809523809523)

('General', 'crowd_number', 0.3214285714285714)

('Tell me how many people were wearing white', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('how', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('were', 'VBD'), ('wearing', 'VBG'), ('white', 'JJ')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'object': 'people', 'properties': []})

```
014 "Which robot is used in the Domestic Standard Platform League" "The Toyota Human Support Robot" [p] predefined
```
answer: The first competition took place in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_objects', 0.4103386809269162)

The first competition took place in 2013

```
015 "How many doors has the kitchen" "" [a] arena_number
```

answer: answer: There are $answer kitchen in .

('Predefined', 'WHAT YEAR IS IT', 0.14285714285714285)

('General', 'arena_number', 0.547943722943723)

('How many doors has the kitchen', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('doors', 'NNS'), ('has', 'VBZ'), ('the', 'DT'), ('kitchen', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'kitchen', 'properties': []})

```
016 "How many elders are in the crowd" [r] crowd_number
```

answer: answer: There are  is elders in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.2734339190221542)

('How many elders are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('elders', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'elders', 'properties': []})

```
017 "Tell me how many people were wearing red" "" [r] crowd_number
```
answer: answer: There are me is people in .

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.09523809523809523)

('General', 'crowd_number', 0.3214285714285714)

('Tell me how many people were wearing red', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('how', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('were', 'VBD'), ('wearing', 'VBG'), ('red', 'JJ')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'object': 'people', 'properties': []})

```
018 "Which is the heaviest object" "" [o] object_trivia
```
answer: answer: Theheaviest object is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 1.0636363636363635)

('Which is the heaviest object', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('heaviest', 'JJS'), ('object', 'NN')])

The $properties $object is $answer.

('Object: ', {'object': 'object', 'properties': ['heaviest']})

```
019 "What time is it" "..." [p] predefined
```

answer: The time is 12:58

('Predefined', 'WHAT TIME IS IT', 0.47406015037593985)
('General', 'crowd_age', 0.07692307692307693)
The time is 12:58

```
020 "What is the colour of the pringles" [o] object_color
```
answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
021 "How many women are in the crowd" [r] crowd_number
```

answer: answer: There are  is women in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.2734339190221542)

('How many women are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('women', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'women', 'properties': []})

```
022 "Where is located the cabinet" [a] object_storage
```
answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I was created in Barcelona

```
023 "How many people in the crowd are pointing left" [r] crowd_number
```
answer: answer: There are  is people in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.3984339190221542)

('How many people in the crowd are pointing left', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN'), ('are', 'VBP'), ('pointing', 'VBG'), ('left', 'VBD')])

There are $properties $person $object in $location.

('Object: ', {'object': 'people', 'properties': []})

```
024 "How many boys are in the crowd" [r] corwd_number
```
answer: answer: There are  is boys in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.2734339190221542)

('How many boys are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('boys', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'boys', 'properties': []})

```
025 "Where can I find a toiletries" [o] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I am from Barcelona

```
026 "In which room is the bookcase" [a] object_storage
```

answer: answer: There are $answer bookcase in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the bookcase', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('bookcase', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'bookcase', 'properties': []})

```
027 "Where does the term computer bug come from" " From a moth trapped in a relay" [p] predefined
```
answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.1181818181818182)

I was created in Barcelona

```
028 "Which is the lightest food" [o] object_trivia
```
answer: answer: Thelightest food is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 2.018181818181818)

('Which is the lightest food', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('lightest', 'JJS'), ('food', 'NN')])

The $properties $object is $answer.

('Object: ', {'object': 'food', 'properties': ['lightest']})

```
029 "Who invented the C programming language" "Ken Thompson and Dennis Ritchie" [p] predefined
```

answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.14285714285714285)

('General', 'arena_objects', 0.01818181818181818)

We have 2017

```
030 "Where is located the cupboard" [a] object_storage
```
answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_classify', 0.2181818181818182)

I was created in Barcelona

```
031 "Tell me how many people were wearing red" [r] crowd_number
```

answer: answer: There are me is people in .

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.09523809523809523)

('General', 'crowd_number', 0.3214285714285714)

('Tell me how many people were wearing red', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('how', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('were', 'VBD'), ('wearing', 'VBG'), ('red', 'JJ')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'object': 'people', 'properties': []})

```
032 "In which room is the cabinet" [a] object_storage
```

answer: answer: There are $answer cabinet in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the cabinet', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('cabinet', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'cabinet', 'properties': []})

```
033 "Where is located the dining table" [a] object_storage
```

answer: answer: There are $answer table in .

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'arena_number', 1.268181818181818)

('Where is located the dining table', 'arena_number')

('Tags:', [('Where', 'WRB'), ('is', 'VBZ'), ('located', 'VBN'), ('the', 'DT'), ('dining', 'NN'), ('table', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'table', 'properties': []})

```
034 "What day is today" "..." [p] predefined
```

answer: Today is Friday, 07 of July

('Predefined', 'WHAT DAY IS IT', 0.37406015037593987)

('General', 'crowd_age', 0.07692307692307693)

Today is Friday, 07 of July

```
035 "How many doors has the kitchen" [a] arena_number
```

answer: answer: There are $answer kitchen in .

('Predefined', 'WHAT YEAR IS IT', 0.14285714285714285)

('General', 'arena_number', 0.547943722943723)

('How many doors has the kitchen', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('doors', 'NNS'), ('has', 'VBZ'), ('the', 'DT'), ('kitchen', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'kitchen', 'properties': []})

```
036 "To which category belong the flakes" [o] object_classify
```

answer: answer: The objects in the flakes are $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_classify', 1.518181818181818)

('To which category belong the flakes', 'object_classify')

('Tags:', [('To', 'TO'), ('which', 'WDT'), ('category', 'NN'), ('belong', 'IN'), ('the', 'DT'), ('flakes', 'NNS')])

The objects in $location are $answer.

('Object: ', {})

```
037 "Tell me the number of adults in the crowd" [r] crowd_number
```

answer: answer: There are me is adults in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.3285204991087345)

('Tell me the number of adults in the crowd', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('the', 'DT'), ('number', 'NN'), ('of', 'IN'), ('adults', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'owner': 'adults', 'object': 'adults', 'properties': []})

```
038 "Was the standing person man or woman" [r] crowd_gender
```

answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 2.8015151515151517)

('Was the standing person man or woman', 'crowd_gender')

('Tags:', [('Was', 'IN'), ('the', 'DT'), ('standing', 'VBG'), ('person', 'NN'), ('man', 'NN'), ('or', 'CC'), ('woman', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'woman', 'properties': []})

```
039 "Where is located the bed" [a] object_storage
```
answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I am from Barcelona

```
040 "In which room is the living table" [a] object_storage
```

answer: answer: There are $answer table in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.3270053475935828)

('In which room is the living table', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('living', 'NN'), ('table', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'table', 'properties': []})

```
041 "How many males are in the crowd" [r] crownd_number
```
answer: answer: There are  is males in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 2.273433919022154)

('How many males are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('males', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'males', 'properties': []})

```
042 "How many people in the crowd are rising left arm" [r] crowd_number
```
answer: answer: There are  is arm in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 3.398433919022154)

('How many people in the crowd are rising left arm', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN'), ('are', 'VBP'), ('rising', 'VBG'), ('left', 'VBD'), ('arm', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'arm', 'properties': []})

```
043 "Tell me how many people were wearing red" [r] corwd_number
```

answer: answer: There are me is people in .

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.09523809523809523)

('General', 'crowd_number', 0.3214285714285714)

('Tell me how many people were wearing red', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('how', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('were', 'VBD'), ('wearing', 'VBG'), ('red', 'JJ')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'object': 'people', 'properties': []})

```
044 "How many doors has the kitchen" [a] arena_number
```
answer: answer: There are $answer kitchen in .

('Predefined', 'WHAT YEAR IS IT', 0.14285714285714285)

('General', 'arena_number', 0.547943722943723)

('How many doors has the kitchen', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('doors', 'NNS'), ('has', 'VBZ'), ('the', 'DT'), ('kitchen', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'kitchen', 'properties': []})

```
045 "Do the melon and mug belong to the same category" [o] object_classify
```

answer: answer: The objects in the same category are $answer.

('Predefined', 'WHAT YEAR IS IT', 0.2857142857142857)

('General', 'object_classify', 1.9863636363636363)

('Do the melon and mug belong to the same category', 'object_classify')

('Tags:', [('Do', 'VB'), ('the', 'DT'), ('melon', 'NN'), ('and', 'CC'), ('mug', 'NN'), ('belong', 'NN'), ('to', 'TO'), ('the', 'DT'), ('same', 'JJ'), ('category', 'NN')])

The objects in $location are $answer.

('Object: ', {'object': 'belong', 'properties': []})

```
046 "How many people live in the Japan" "A little over 80 million" [p] predefined
```
answer: RoboCup At Home was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 0.3984339190221543)

RoboCup At Home was founded in 2013

```
047 "How many cupboard are in the kitchen" [a] arena_number
```
answer: answer: There are $answer cupboard in the kitchen.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 0.8692672523554876)

('How many cupboard are in the kitchen', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('cupboard', 'NN'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('kitchen', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'cupboard', 'properties': []})

```
048 "How many people in the crowd are rising left arm" [r] crowd_number
```

answer: answer: There are  is arm in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 3.398433919022154)

('How many people in the crowd are rising left arm', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN'), ('are', 'VBP'), ('rising', 'VBG'), ('left', 'VBD'), ('arm', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'arm', 'properties': []})

```
049 "Where is located the cupboard" [a] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_classify', 0.2181818181818182)

I am from Barcelona

```
050 "What is the colour of the tea" [o] object_color
```
answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
051 "When was invented the C programming language" "C was developed after B in 1972 at Bell Labs" [p] predefined
```

answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.75)

('General', 'crowd_gender', 1.018181818181818)

('When was invented the C programming language', 'crowd_gender')

('Tags:', [('When', 'WRB'), ('was', 'VBD'), ('invented', 'VBN'), ('the', 'DT'), ('C', 'NNP'), ('programming', 'NN'), ('language', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'language', 'properties': []})

```
052 "In which room is the sink" [o] object_storage
```
answer: answer: There are $answer sink in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the sink', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('sink', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'sink', 'properties': []})

```
053 "In which room is the living table" [a] object_storage
```

answer: answer: There are $answer table in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.3270053475935828)

('In which room is the living table?', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('living', 'VBG'), ('table', 'NN'), ('?', '.')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'table', 'properties': []})

```
054 "Tell me if the rising right arm person was a male" [r] crowd_gender
```

answer: answer: There are me is male in the rising.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_number', 3.018181818181818)

('Tell me if the rising right arm person was a male', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('if', 'IN'), ('the', 'DT'), ('rising', 'VBG'), ('right', 'RB'), ('arm', 'JJ'), ('person', 'NN'), ('was', 'VBD'), ('a', 'DT'), ('male', 'NN')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'object': 'male', 'properties': []})

```
055 "How many elders are in the crowd" [r] crownd_number
```

answer: answer: There are  is elders in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.2734339190221542)

('How many elders are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('elders', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'elders', 'properties': []})

```
056 "Was the lying person male or female" [r] crowd_gender
```

answer: answer: There are  is female in .

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_number', 3.268181818181818)

('Was the lying person male or female', 'crowd_number')

('Tags:', [('Was', 'VBD'), ('the', 'DT'), ('lying', 'VBG'), ('person', 'NN'), ('male', 'NN'), ('or', 'CC'), ('female', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'female', 'properties': []})

```
057 "Tell me the number of adults in the crowd" [r] crowd_number
```

answer: answer: There are me is adults in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.3285204991087345)

('Tell me the number of adults in the crowd', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('the', 'DT'), ('number', 'NN'), ('of', 'IN'), ('adults', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'owner': 'adults', 'object': 'adults', 'properties': []})

```
058 "Where is located the bar" [a] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I was created in Barcelona

```
059 "How many doors has the bedroom" [a] arena_number
```

answer: answer: There are $answer bedroom in .

('Predefined', 'WHAT YEAR IS IT', 0.14285714285714285)

('General', 'arena_number', 0.2146103896103896)

('How many doors has the bedroom', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('doors', 'NNS'), ('has', 'VBZ'), ('the', 'DT'), ('bedroom', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'bedroom', 'properties': []})

```
060 "In which room is the bar" [a] object_storage
```
answer: answer: There are $answer bar in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the bar', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('bar', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'bar', 'properties': []})

```
061 "Which is the lightest snacks" [o] object_trivia
```

answer: answer: Thelightest snacks is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 1.018181818181818)

('Which is the lightest snacks', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('lightest', 'JJS'), ('snacks', 'NNS')])

The $properties $object is $answer.

('Object: ', {'object': 'snacks', 'properties': ['lightest']})

```
062 "In which room is the dining table" [a] object_storage
```

answer: answer: There are $answer table in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 2.327005347593583)

('In which room is the dining table', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('dining', 'NN'), ('table', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'table', 'properties': []})

```
063 "Was the sitting person man or woman" [r] crowd_gender
```
answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 3.3015151515151517)

('Was the sitting person man or woman', 'crowd_gender')

('Tags:', [('Was', 'VBD'), ('the', 'DT'), ('sitting', 'VBG'), ('person', 'NN'), ('man', 'NN'), ('or', 'CC'), ('woman', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'woman', 'properties': []})

```
064 "What is a Sakura" "Sakura is the Japanese term for ornamental cherry blossom and its tree" [p] predefined
```
answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.12406015037593984)

('General', 'crowd_gender', 0.21978021978021978)

('What is a Sakura', 'crowd_gender')

('Tags:', [('What', 'WP'), ('is', 'VBZ'), ('a', 'DT'), ('Sakura', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'Sakura', 'properties': []})

```
065 "How many people in the crowd are rising right arm" [r] crowd_number
```
answer: answer: There are  is arm in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 3.398433919022154)

('How many people in the crowd are rising right arm', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN'), ('are', 'VBP'), ('rising', 'VBG'), ('right', 'RB'), ('arm', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'arm', 'properties': []})

```
066 "Where is located the sink" [o] object_storage
```
answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I was created in Barcelona

```
067 "Do the bowl and shampoo belong to the same category" [o] object_classify
```
answer: answer: The objects in the same category are $answer.

('Predefined', 'WHAT YEAR IS IT', 0.2857142857142857)

('General', 'object_classify', 1.9863636363636363)

('Do the bowl and shampoo belong to the same category', 'object_classify')

('Tags:', [('Do', 'VB'), ('the', 'DT'), ('bowl', 'NN'), ('and', 'CC'), ('shampoo', 'NN'), ('belong', 'NN'), ('to', 'TO'), ('the', 'DT'), ('same', 'JJ'), ('category', 'NN')])

The objects in $location are $answer.

('Object: ', {'object': 'belong', 'properties': []})

```
068 "Tell me how many people were wearing yellow" [r] crowd_number
```
answer: answer: There are me is yellow in .

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.09523809523809523)

('General', 'crowd_number', 0.3214285714285714)

('Tell me how many people were wearing yellow', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('how', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('were', 'VBD'), ('wearing', 'VBG'), ('yellow', 'NN')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'object': 'yellow', 'properties': []})

```
069 "Tell me if the pointing right person was a man" [r] crowd_gender
```
answer: answer: Sorry, I cannot answer that question. I do not know what gender me is.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 2.4943722943722944)

('Tell me if the pointing right person was a man', 'crowd_gender')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('if', 'IN'), ('the', 'DT'), ('pointing', 'VBG'), ('right', 'JJ'), ('person', 'NN'), ('was', 'VBD'), ('a', 'DT'), ('man', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('me', 'PRP')

('Object: ', {'object': 'man', 'properties': []})

```
070 "Where is located the living table" [a] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'arena_objects', 0.2681818181818182)

I am from Barcelona

```
071 "What are the colours of the Japanese flag" "Japanese flag is a red circle centred over white" [p] predefined
```
answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.3383458646616541)

('General', 'arena_objects', 0.23219696969696968)

We have 2017


```
072 "Where is located the bed" [a] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I was created in Barcelona

```
073 "Tell me the number of elders in the crowd" [r] crowd_number
```

answer: answer: There are me is elders in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.3285204991087345)

('Tell me the number of elders in the crowd', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('the', 'DT'), ('number', 'NN'), ('of', 'IN'), ('elders', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'owner': 'elders', 'object': 'elders', 'properties': []})

```
074 "How many bed are in the bedroom" [a] arena_number
```

answer: It was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 0.3359339190221543)

It was founded in 2013

```
075 "Was the standing person boy or girl" [r] crowd_gender
```

answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 2.268181818181818)

('Was the standing person boy or girl', 'crowd_gender')

('Tags:', [('Was', 'IN'), ('the', 'DT'), ('standing', 'VBG'), ('person', 'NN'), ('boy', 'NN'), ('or', 'CC'), ('girl', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'girl', 'properties': []})

```
076 "In which room is the desk" [a] object_storage
```

answer: answer: There are $answer desk in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the desk', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('desk', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'desk', 'properties': []})

```
077 "How many snacks there are" [o] arena_number
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'arena_number', 0.2589285714285714)

I am from Barcelona

```
078 "Where can I find a flakes" [o] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I was created in Barcelona


```
079 "How many women are in the crowd" [r] crowd_number
```

answer: answer: There are  is women in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.2734339190221542)

(' How many women are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('women', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'women', 'properties': []})

```
080 "Which is the smallest containers" [o] object_trivia
```

answer: answer: Thesmallest containers is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 1.018181818181818)

(' Which is the smallest containers', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('smallest', 'JJS'), ('containers', 'NNS')])

The $properties $object is $answer.
('Object: ', {'object': 'containers', 'properties': ['smallest']})

```
081 "How many people live in the Japan" "A little over 80 million" [p] predefined
```
answer: It was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 0.3984339190221543)

It was founded in 2013

```
082 "How many desk are in the corridor" [a] arena_number
```

answer: RoboCup At Home was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 0.3359339190221543)

RoboCup At Home was founded in 2013

```
083 "In which room is the desk" [a] object_storage
```

answer: answer: There are $answer desk in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the desk', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('desk', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'desk', 'properties': []})

```
084 "How many girls are in the crowd" [r] crowd_number
```

answer: answer: There are  is girls in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.2734339190221542)

('How many girls are in the crowd', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('girls', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('Object: ', {'object': 'girls', 'properties': []})

```
085 "How many candies are in the bar" [o] arena_objects
```

answer: The first competition took place in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 0.3359339190221543)

The first competition took place in 2013

```
086 "In which room is the TV stand" [a] object_storage
```

answer: answer: There are $answer stand in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the TV stand?', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('TV', 'NN'), ('stand', 'NN'), ('?', '.')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'stand', 'properties': []})

```
087 "Between the bowl and peanuts, which one is lighter" [o] object_trivia
```

answer: answer: There are $answer in the bowl.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_objects', 0.6848484848484848)

('Between the bowl and peanuts, which one is lighter', 'arena_objects')

('Tags:', [('Between', 'IN'), ('the', 'DT'), ('bowl', 'NN'), ('and', 'CC'), ('peanuts', 'NNS'), (',', ','), ('which', 'WDT'), ('one', 'CD'), ('is', 'VBZ'), ('lighter', 'JJR')])

There are $answer in $location.

('Object: ', {'object': 'peanuts', 'properties': []})

```
088 "Which robot is used in the Domestic Standard Platform League" "The Toyota Human Support Robot" [p] predefined
```

answer: It was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_objects', 0.4103386809269162)

It was founded in 2013

```
089 "Where is located the center table" [a] object_storage
```

answer: I am from Barcelon

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'arena_objects', 0.2681818181818182)

I am from Barcelona

```
090 "Was the lying person man or woman" [r] crowd_gender
```

answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 2.8015151515151517)

('Was the lying person man or woman', 'crowd_gender')

('Tags:', [('Was', 'VBD'), ('the', 'DT'), ('lying', 'VBG'), ('person', 'NN'), ('man', 'NN'), ('or', 'CC'), ('woman', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'woman', 'properties': []})

```
091 "To which category belong the glass" [o] object_classify
```

answer: answer: The objects in the glass are $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_classify', 1.518181818181818)

('To which category belong the glass', 'object_classify')

('Tags:', [('To', 'TO'), ('which', 'WDT'), ('category', 'NN'), ('belong', 'IN'), ('the', 'DT'), ('glass', 'NN')])

The objects in $location are $answer.

('Object: ', {})

```
092 "Which is the biggest containers" [o] object_trivia
```

answer: answer: Thebiggest containers is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 1.018181818181818)

('Which is the biggest containers', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('biggest', 'JJS'), ('containers', 'NNS')])

The $properties $object is $answer.

('Object: ', {'object': 'containers', 'properties': ['biggest']})

```
093 "Do you have dreams" "I dream of Electric Sheep" [p] predefined
```

answer: answer: Sorry, I cannot answer that question. I do not know what age  is.

('Predefined', 'WHAT IS YOUR TEAMS NAME', 0.16666666666666666)

('General', 'crowd_age', 0.3666666666666667)

('Do you have dreams?', 'crowd_age')

('Tags:', [('Do', 'VBP'), ('you', 'PRP'), ('have', 'VB'), ('dreams', 'NNS'), ('?', '.')])

Sorry, I cannot answer that question. I do not know what age $person.

('Object: ', {'object': 'dreams', 'properties': []})

```
094 "How many toiletries there are" [o] arena_number
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'arena_number', 0.2589285714285714)

I was created in Barcelona

```
095 "What city is the capital of the Japan?" "Tokyo" [p] predefined
```
answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
096 "Tell me if the sitting person was a man" [r] crowd_gender
```
answer: answer: Sorry, I cannot answer that question. I do not know what gender me is.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 2.9943722943722944)

('Tell me if the sitting person was a man', 'crowd_gender')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('if', 'IN'), ('the', 'DT'), ('sitting', 'VBG'), ('person', 'NN'), ('was', 'VBD'), ('a', 'DT'), ('man', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('me', 'PRP')

('Object: ', {'object': 'man', 'properties': []})

```
097 "How many living table are in the bedroom" [a] arena_number
```

answer: answer: There are $answer table in the bedroom.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 0.5859339190221543)

('How many living table are in the bedroom', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('living', 'NN'), ('table', 'NN'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('bedroom', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'table', 'properties': []})

```
098 "What objects are stored in the bookcase" [o] arena_objects
```
answer: RoboCup At Home was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.2681818181818182)

RoboCup At Home was founded in 2013

```
099 "In which room is the fridge" [a] object_storage
```

answer: answer: There are $answer fridge in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the fridge', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('fridge', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'fridge', 'properties': []})

```
100 "Was the lying person man or woman" [r] crowd_gender
```

answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'WHAT WAS THE LAST ANSWER', 0.39285714285714285)

('General', 'crowd_gender', 2.8015151515151517)

('Was the lying person man or woman', 'crowd_gender')

('Tags:', [('Was', 'VBD'), ('the', 'DT'), ('lying', 'VBG'), ('person', 'NN'), ('man', 'NN'), ('or', 'CC'), ('woman', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'woman', 'properties': []})


```
101 "In which room is the cabinet" [a] object_storage
```
answer: answer: There are $answer cabinet in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the cabinet', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('cabinet', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'cabinet', 'properties': []})

```
102 "Where does the term computer bug come from" "From a moth trapped in a relay" [p] predefined
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.1181818181818182)

I am from Barcelona

```
103 "Where can I find a candies" [o] object_storage
```
answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I was created in Barcelona

```
104 "Where is located the cabinet" [a] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I was created in Barcelona

```
105 "Tell me the number of females in the crowd" [r] crowd_gender
```
answer: answer: There are me is females in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 2.328520499108734)

('Tell me the number of females in the crowd', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('the', 'DT'), ('number', 'NN'), ('of', 'IN'), ('females', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'owner': 'females', 'object': 'females', 'properties': []})

```
106 "What objects are stored in the bedside" [o] arena_objects
```
answer: RoboCup At Home was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.2681818181818182)

RoboCup At Home was founded in 201

```
107 "How many people in the crowd are sitting or lying" [r] crowd_number
```

answer: answer: There are  is people in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 3.148433919022154)

('How many people in the crowd are sitting or lying', 'crowd_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('people', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN'), ('are', 'VBP'), ('sitting', 'VBG'), ('or', 'CC'), ('lying', 'VBG')])

There are $properties $person $object in $location.

('Object: ', {'object': 'people', 'properties': []})

```
108 "When was invented the B programming language" "B was developed circa 1969 at Bell Labs" [p] predefined
```
answer: answer: Sorry, I cannot answer that question. I do not know what gender  is.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.75)

('General', 'crowd_gender', 1.018181818181818)

('When was invented the B programming language', 'crowd_gender')

('Tags:', [('When', 'WRB'), ('was', 'VBD'), ('invented', 'VBN'), ('the', 'DT'), ('B', 'NNP'), ('programming', 'NN'), ('language', 'NN')])

Sorry, I cannot answer that question. I do not know what gender $person.

('Object: ', {'object': 'language', 'properties': []})

```
109 "Tell me the number of children in the crowd" [r] crowd_number
```

answer: answer: There are me is children in the crowd.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 2.328520499108734)

('Tell me the number of children in the crowd', 'crowd_number')

('Tags:', [('Tell', 'VB'), ('me', 'PRP'), ('the', 'DT'), ('number', 'NN'), ('of', 'IN'), ('children', 'NNS'), ('in', 'IN'), ('the', 'DT'), ('crowd', 'NN')])

There are $properties $person $object in $location.

('me', 'PRP')

('Object: ', {'owner': 'children', 'object': 'children', 'properties': []})

```
110 "To which category belong the cloth" [o] object_classfiy
```
answer: answer: The objects in the cloth are $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_classify', 1.518181818181818)

('To which category belong the cloth', 'object_classify')

('Tags:', [('To', 'TO'), ('which', 'WDT'), ('category', 'NN'), ('belong', 'IN'), ('the', 'DT'), ('cloth', 'NN')])

The objects in $location are $answer.

('Object: ', {})

```
111 "What objects are stored in the living shelf" [o] arena_objects
```
answer: The first competition took place in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.4681818181818182)

The first competition took place in 2013

```
112 "Do the mints and soap belong to the same category" [o] object_classify
```

answer: answer: The objects in the same category are $answer.

('Predefined', 'WHAT YEAR IS IT', 0.2857142857142857)

('General', 'object_classify', 1.9863636363636363)

('Do the mints and soap belong to the same category', 'object_classify')

('Tags:', [('Do', 'VB'), ('the', 'DT'), ('mints', 'NNS'), ('and', 'CC'), ('soap', 'NN'), ('belong', 'NN'), ('to', 'TO'), ('the', 'DT'), ('same', 'JJ'), ('category', 'NN')])

The objects in $location are $answer.

('Object: ', {'object': 'belong', 'properties': []})

```
113 "What is the colour of the chocolate bar" [o] object_color
```
answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
114 "What objects are stored in the bar" [o] arena_objects
```
answer: The first competition took place in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.2681818181818182)

The first competition took place in 2013

```
115 "Do the peanuts and cloth belong to the same category" [o] object_classify
```
answer: answer: The objects in the same category are $answer.

('Predefined', 'WHAT YEAR IS IT', 0.2857142857142857)

('General', 'object_classify', 1.9863636363636363)

('Do the peanuts and cloth belong to the same category', 'object_classify')

('Tags:', [('Do', 'VB'), ('the', 'DT'), ('peanuts', 'NNS'), ('and', 'CC'), ('cloth', 'VB'), ('belong', 'NN'), ('to', 'TO'), ('the', 'DT'), ('same', 'JJ'), ('category', 'NN')])

The objects in $location are $answer.

('Object: ', {'object': 'belong', 'properties': []})

```
116 "To which category belong the apple" [o] object_classify
```
answer: answer: The objects in the apple are $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_classify', 1.518181818181818)

('To which category belong the apple', 'object_classify')

('Tags:', [('To', 'TO'), ('which', 'WDT'), ('category', 'NN'), ('belong', 'IN'), ('the', 'DT'), ('apple', 'NN')])

The objects in $location are $answer.

('Object: ', {})

```
117 "Between the cloth and soap, which one is bigger" [o] object_trivia
```

answer: answer: The soap is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_properties', 1.0951048951048952)

('Between the cloth and soap, which one is bigger', 'object_properties')

('Tags:', [('Between', 'IN'), ('the', 'DT'), ('cloth', 'NN'), ('and', 'CC'), ('soap', 'NN'), (',', ','), ('which', 'WDT'), ('one', 'CD'), ('is', 'VBZ'), ('bigger', 'JJR')])

The $object is $answer.

('Object: ', {'object': 'soap', 'properties': []})

```
118 "Do the cloth and paprika belong to the same category" [o] object_classify
```

answer: answer: The objects in the same category are $answer.

('Predefined', 'WHAT YEAR IS IT', 0.2857142857142857)

('General', 'object_classify', 1.9863636363636363)

('Do the cloth and paprika belong to the same category', 'object_classify')

('Tags:', [('Do', 'VB'), ('the', 'DT'), ('cloth', 'NN'), ('and', 'CC'), ('paprika', 'VB'), ('belong', 'NN'), ('to', 'TO'), ('the', 'DT'), ('same', 'JJ'), ('category', 'NN')])

The objects in $location are $answer.

('Object: ', {'object': 'belong', 'properties': []})


```
119 "Where can I find a noodles" [o] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I was created in Barcelona

```
120 "What is the colour of the mug" [o] object_color
```
answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
121 "How many food are in the dining table" [o] arena_number
```

answer: answer: There are $answer food in the dining table.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 1.5859339190221542)

('How many food are in the dining table', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('food', 'NN'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('dining', 'NN'), ('table', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'food', 'properties': []})

```
122 "Where can I find a containers" [o] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I was created in Barcelona

```
123 "What objects are stored in the counter" [o] arena_objects
```

answer: The first competition took place in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.2681818181818182)

The first competition took place in 2013

```
124 "How many food there are" [o] arena_number
```
answer: answer: The food is $answer.

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_trivia', 1.0)

('How many food there are', 'object_trivia')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('food', 'NN'), ('there', 'EX'), ('are', 'VBP')])

The $properties $object is $answer.

('Object: ', {'object': 'food', 'properties': []})

```
125 "What's the colour of the sake" [o] object_color
```

answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
126 "To which category belong the chocolate egg" [o] object_classify
```

answer: answer: The objects in the chocolate egg are $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_classify', 1.518181818181818)

('To which category belong the chocolate egg', 'object_classify')

('Tags:', [('To', 'TO'), ('which', 'WDT'), ('category', 'NN'), ('belong', 'IN'), ('the', 'DT'), ('chocolate', 'NN'), ('egg', 'NN')])

The objects in $location are $answer.

('Object: ', {})

```
127 "To which category belong the glass" [o] object_classify
```

answer: answer: The objects in the glass are $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_classify', 1.518181818181818)

('To which category belong the glass', 'object_classify')

('Tags:', [('To', 'TO'), ('which', 'WDT'), ('category', 'NN'), ('belong', 'IN'), ('the', 'DT'), ('glass', 'NN')])

The objects in $location are $answer.

('Object: ', {})

```
128 "Between the noodles and oat, which one is smaller" [o] object_trivia
```

answer: answer: There are $answer in the noodles.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_objects', 0.6848484848484848)

('Between the noodles and oat, which one is smaller', 'arena_objects')

('Tags:', [('Between', 'IN'), ('the', 'DT'), ('noodles', 'NNS'), ('and', 'CC'), ('oat', 'NN'), (',', ','), ('which', 'WDT'), ('one', 'CD'), ('is', 'VBZ'), ('smaller', 'JJR')])

There are $answer in $location.

('Object: ', {'object': 'oat', 'properties': []})

```
129 "Where can I find a sushi" [o] object_storage
```
answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I am from Barcelona

```
130 "How many objects are in the dining table" [o] arena_number
```
answer: answer: There are $answer objects in the dining table

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 1.6313884644766998)

('How many objects are in the dining table', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('objects', 'NNS'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('dining', 'NN'), ('table', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'objects', 'properties': []})

```
131 "Do the paprika and cloth belong to the same category" [o] object_classify
```

answer: answer: The objects in the same category are $answer.

('Predefined', 'WHAT YEAR IS IT', 0.2857142857142857)

('General', 'object_classify', 1.9863636363636363)

('Do the paprika and cloth belong to the same category', 'object_classify')

('Tags:', [('Do', 'VB'), ('the', 'DT'), ('paprika', 'NN'), ('and', 'CC'), ('cloth', 'NN'), ('belong', 'NN'), ('to', 'TO'), ('the', 'DT'), ('same', 'JJ'), ('category', 'NN')])

The objects in $location are $answer.

('Object: ', {'object': 'belong', 'properties': []})

```
132 "Where can I find a cloth" [o] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I am from Barcelona

```
133 "What is the colour of the sponge" [o] object_color
```
answer: answer: The sponge is $answer.

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'object_color', 1.1696969696969697)

('What is the colour of the sponge', 'object_color')

('Tags:', [('What', 'WP'), ('is', 'VBZ'), ('the', 'DT'), ('colour', 'NN'), ('of', 'IN'), ('the', 'DT'), ('sponge', 'NN')])

The $object is $answer.

('Object: ', {'owner': 'the', 'object': 'sponge', 'properties': []})

```
134 "Where can I find a soap" [o] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I am from Barcelona

```
135 "Where can I find a beer" [o] object_storage
```
answer: answer: The beer is stored in the $answer.

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 1.6178571428571429)

('Where can I find a beer', 'object_storage')

('Tags:', [('Where', 'WRB'), ('can', 'MD'), ('I', 'PRP'), ('find', 'VB'), ('a', 'DT'), ('beer', 'NN')])

The $object is stored in the $answer.

('I', 'PRP')

('Object: ', {'object': 'beer', 'properties': []})

```
136 "Between the chocolate bar and shampoo, which one is smaller" [o] object_trivia
```
answer: answer: The shampoo is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_color', 1.018181818181818)

('Between the chocolate bar and shampoo, which one is smaller', 'object_color')

('Tags:', [('Between', 'IN'), ('the', 'DT'), ('chocolate', 'NN'), ('bar', 'NN'), ('and', 'CC'), ('shampoo', 'NN'), (',', ','), ('which', 'WDT'), ('one', 'CD'), ('is', 'VBZ'), ('smaller', 'JJR')])

The $object is $answer.

('Object: ', {'object': 'shampoo', 'properties': []})

```
137 "Where can I find a senbei" [o] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I am from Barcelona

```
138 "How many snacks there are" [o] arena_number
```
answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'arena_number', 0.2589285714285714)

I am from Barcelona

```
139 "Which is the heaviest object" [o] object_trivia
```

answer: answer: Theheaviest object is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 1.0636363636363635)

('Which is the heaviest object', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('heaviest', 'JJS'), ('object', 'NN')])

The $properties $object is $answer.

('Object: ', {'object': 'object', 'properties': ['heaviest']})


```
140 "Between the paprika and apple, which one is heavier" [o] object_trivia
```
answer: answer: There are $answer in the paprika.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_objects', 0.6848484848484848)

('Between the paprika and apple, which one is heavier', 'arena_objects')

('Tags:', [('Between', 'IN'), ('the', 'DT'), ('paprika', 'NN'), ('and', 'CC'), ('apple', 'NN'), (',', ','), ('which', 'WDT'), ('one', 'CD'), ('is', 'VBZ'), ('heavier', 'JJR')])

There are $answer in $location.

('Object: ', {'object': 'apple', 'properties': []})

```
141 "To which category belong the coke" [o] object_classify
```

answer: answer: The objects in the coke are $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_classify', 1.518181818181818)

('To which category belong the coke', 'object_classify')

('Tags:', [('To', 'TO'), ('which', 'WDT'), ('category', 'NN'), ('belong', 'IN'), ('the', 'DT'), ('coke', 'NN')])

The objects in $location are $answer.

('Object: ', {})

```
142 "Where can I find a pringles" [o] object_storage
```
answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.6666666666666666)

('General', 'object_storage', 0.6178571428571429)

I am from Barcelona

```
143 "Which is the biggest toiletries" [o] object_trivia
```

answer: answer: Thebiggest toiletries is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 1.018181818181818)

('Which is the biggest toiletries', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('biggest', 'JJS'), ('toiletries', 'NNS')])

The $properties $object is $answer.

('Object: ', {'object': 'toiletries', 'properties': ['biggest']})

```
144 "What objects are stored in the TV stand" [o] arena_objects
```
answer: answer: There are  is unknown object in the TV stand.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'crowd_number', 1.0770053475935828)

('What objects are stored in the TV stand', 'crowd_number')

('Tags:', [('What', 'WP'), ('objects', 'VBZ'), ('are', 'VBP'), ('stored', 'VBN'), ('in', 'IN'), ('the', 'DT'), ('TV', 'NN'), ('stand', 'NN')])

There are $properties $person $object in $location.

('Object: ', {})

```
145 "How many objects are in the drawer" [o] arena_number
```

answer: RoboCup At Home was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 0.3813884644766998)

RoboCup At Home was founded in 2013

```
146 "What objects are stored in the cabinet" [o] arena_objects
```

answer: RoboCup At Home was founded in 2013

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_storage', 0.2681818181818182)

RoboCup At Home was founded in 2013


```
147 "Which is the smallest object" [o] object_trivia
```

answer: answer: Thesmallest object is $answer.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'object_trivia', 1.0636363636363635)

('Which is the smallest object', 'object_trivia')

('Tags:', [('Which', 'WDT'), ('is', 'VBZ'), ('the', 'DT'), ('smallest', 'JJS'), ('object', 'NN')])

The $properties $object is $answer.

('Object: ', {'object': 'object', 'properties': ['smallest']})

```
148 "What objects are stored in the center table" [o] arena_objects
```

answer: answer: There are $answer in the center table.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_objects', 0.5016265597147951)

('What objects are stored in the center table', 'arena_objects')

('Tags:', [('What', 'WP'), ('objects', 'VBZ'), ('are', 'VBP'), ('stored', 'VBN'), ('in', 'IN'), ('the', 'DT'), ('center', 'NN'), ('table', 'NN')])

There are $answer in $location.

('Object: ', {})

```
149 "What's the colour of the chips" [o] object_color
```
answer: We have 2017

('Predefined', 'WHAT YEAR IS IT', 0.4097744360902255)

('General', 'arena_objects', 0.16969696969696968)

We have 2017

```
150 "Where is located the drawer" [a] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I was created in Barcelona

```
151 "Where is located the desk" [a] object_storage
```

answer: I am from Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I am from Barcelona

```
152 "Where is located the sideshelf" [a] object_storage
```

answer: I was created in Barcelona

('Predefined', 'WHERE ARE YOU FROM', 0.3333333333333333)

('General', 'object_storage', 0.19510489510489512)

I was created in Barcelona

```
153 "In which room is the bed" [a] object_storage
```
answer: answer: There are $answer bed in the which room.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 1.0)

('General', 'arena_number', 1.0770053475935828)

('In which room is the bed?', 'arena_number')

('Tags:', [('In', 'IN'), ('which', 'WDT'), ('room', 'NN'), ('is', 'VBZ'), ('the', 'DT'), ('bed', 'NN'), ('?', '.')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'bed', 'properties': []})

```
154 "How many living table are in the corridor" [a] arena_number
```

answer: answer: There are $answer table in the corridor.

('Predefined', 'IN WHICH YEAR ROBOCUP AT HOME WAS FOUNDED', 0.5)

('General', 'arena_number', 0.5859339190221543)

('How many living table are in the corridor', 'arena_number')

('Tags:', [('How', 'WRB'), ('many', 'JJ'), ('living', 'NN'), ('table', 'NN'), ('are', 'VBP'), ('in', 'IN'), ('the', 'DT'), ('corridor', 'NN')])

There are $answer $properties $object in $location.

('Object: ', {'object': 'table', 'properties': []})

```
155 "Which robot is used in the Social Standard Platform League" "The SoftBank Robotics Pepper" [p] predefined
```

```
156 "Who invented the C programming language" "Ken Thompson and Dennis Ritchie" [p] predefined
```

```
157 "Which robot is used in the Domestic Standard Platform League" "The Toyota Human Support Robot" [p] predefined
```
```
158 "Who is the emperor of Japan" "His Majesty Akihito sama is the emperor in Japan since January 7, 1989" [p] predefined
```

```
159 "Who invented the first compiler" "Grace Brewster Murray Hopper invented it" [p] predefined
```

```
160 "In which city will next year's RoboCup be hosted" "It hasn't been announced yet" [p] predefined
```

```
161 "Which robot is used in the Social Standard Platform League" "The SoftBank Robotics Pepper" [p] predefined
```
```
162 "When was invented the B programming language" "B was developed circa 1969 at Bell Labs" [p] predefined
```






