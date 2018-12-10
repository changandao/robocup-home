#!/usr/bin/env python

# Uses code from https://chatbotslife.com/text-classification-using-algorithms-e4d50dcba45

import rospy
import os, os.path, sys, time
import time

import nltk
from nltk.stem.lancaster import LancasterStemmer

from std_msgs.msg import String
from tum_alle_athome_speech_msgs.srv import srvQuestion, srvGenerateAnswer

absbasepath = os.path.abspath(os.path.dirname(__file__))

## Class storing basic tokenization methods
#
class Intent_Classifier:
    ## Create a Intent_Classifier class object.
    #
    def __init__(self, filename = 'training_nlp.csv', filename_aiml = 'training_predefined.csv'):
        self.aiml_server = rospy.ServiceProxy('/tiago/speech/predefined_answer', srvQuestion)
        self.nlp_server = rospy.ServiceProxy('/tiago/speech/generate_answer', srvGenerateAnswer)
        ## Punctuation signs
        self.punctuation = ['.', ',', ';', ':']
        ## Stemmer for cutting words
        self.stemmer = LancasterStemmer()
        ## Training data for predefined
        self.aiml_training = []
        # capture unique stemmed words in the predefined training corpus
        self.aiml_corpus_words = {}
        self.aiml_class_words = {}
        ## Training data
        self.training = []
        # capture unique stemmed words in the training corpus
        self.corpus_words = {}
        self.class_words = {}
        # Read training data
        self.read_csv(filename, filename_aiml)
        # Train
        self.train()

    def train(self, show_details = False):
        # turn a list into a set (of unique items) and then a list again (this removes duplicates)
        classes = list(set([a['class'] for a in self.training]))
        for c in classes:
            # prepare a list of words within each class
            self.class_words[c] = []
        # loop through each sentence in our training data
        for data in self.training:
            # tokenize each sentence into words
            for word in nltk.word_tokenize(data['sentence']):
                # ignore a some things
                if word not in ["?", "'s"] and word not in self.punctuation:
                    # stem and lowercase each word
                    stemmed_word = self.stemmer.stem(word.lower())
                    # have we not seen this word already?
                    if stemmed_word not in self.corpus_words:
                        self.corpus_words[stemmed_word] = 1
                    else:
                        self.corpus_words[stemmed_word] += 1
                    # add the word to our words in class list
                    self.class_words[data['class']].extend([stemmed_word])
        if show_details:
            # we now have each stemmed word and the number of occurances of the word in our training corpus (the word's commonality)
            print ("Corpus words and counts: %s \n" % self.corpus_words)
            # also we have all words in each class
            print ("Class words: %s" % self.class_words)
        # turn a list into a set (of unique items) and then a list again (this removes duplicates)
        classes = list(set([a['class'] for a in self.aiml_training]))
        for c in classes:
            # prepare a list of words within each class
            self.aiml_class_words[c] = []
        # loop through each sentence in our training data
        for data in self.aiml_training:
            # tokenize each sentence into words
            for word in nltk.word_tokenize(data['sentence']):
                # ignore a some things
                if word not in ["?", "'s"] and word not in self.punctuation:
                    # stem and lowercase each word
                    stemmed_word = self.stemmer.stem(word.lower())
                    # have we not seen this word already?
                    if stemmed_word not in self.aiml_corpus_words:
                        self.aiml_corpus_words[stemmed_word] = 1
                    else:
                        self.aiml_corpus_words[stemmed_word] += 1
                    # add the word to our words in class list
                    self.aiml_class_words[data['class']].extend([stemmed_word])
        if show_details:
            # we now have each stemmed word and the number of occurances of the word in our training corpus (the word's commonality)
            print ("Corpus words and counts: %s \n" % self.aiml_corpus_words)
            # also we have all words in each class
            print ("Class words: %s" % self.aiml_class_words)

    ## calculate a score for a given class taking into account word commonality
    def calculate_class_score(self, sentence, class_name, show_details=False):
        score = 0
        # tokenize each word in our new sentence
        for word in nltk.word_tokenize(sentence):
            # check to see if the stem of the word is in any of our classes
            if self.stemmer.stem(word.lower()) in self.class_words[class_name]:
                # treat each word with relative weight
                score += float(1 / float(self.corpus_words[self.stemmer.stem(word.lower())]))
                if show_details:
                    print ("   match: %s (%s)" % (self.stemmer.stem(word.lower()), 1 / float(self.corpus_words[self.stemmer.stem(word.lower())])))
        return score

    ## calculate a score for a given class taking into account word commonality
    def calculate_aiml_score(self, sentence, class_name, show_details=False):
        score = 0
        # tokenize each word in our new sentence
        for word in nltk.word_tokenize(sentence):
            # check to see if the stem of the word is in any of our classes
            if self.stemmer.stem(word.lower()) in self.aiml_class_words[class_name]:
                # treat each word with relative weight
                score += float(1 / float(self.aiml_corpus_words[self.stemmer.stem(word.lower())]))
                if show_details:
                    print ("   match: %s (%s)" % (self.stemmer.stem(word.lower()), 1 / float(self.aiml_corpus_words[self.stemmer.stem(word.lower())])))
        return score

    ## return the class with highest score for sentence
    def classify(self, sentence):
        high_class = None
        high_score = 0
        # loop through our classes
        for c in self.class_words.keys():
            # calculate score of sentence for each class
            score = self.calculate_class_score(sentence, c, show_details=False)
            # keep track of highest score
            if score > high_score:
                high_class = c
                high_score = score
        return high_class, high_score

    ## return the class with highest score for sentence
    def classify_aiml(self, sentence):
        high_class = None
        high_score = 0
        # loop through our classes
        for c in self.aiml_class_words.keys():
            # calculate score of sentence for each class
            score = self.calculate_aiml_score(sentence, c, show_details=False)
            # keep track of highest score
            if score > high_score:
                high_class = c
                high_score = score
        return high_class, high_score

    def read_csv(self, filename, filename_aiml):
        path = os.path.join(absbasepath,"../training/") + filename
        print (path)
        f = open(path, 'r')
        # Read the intents and their sentences
        content = f.readlines()
        content = [x.strip() for x in content]
        for line in content:
            text_state_pair = line.split(",")
            temp_dict = {}
            temp_dict['class'] = text_state_pair[0]
            temp_dict['sentence'] = text_state_pair[1]
            self.training.append(temp_dict)
        print ("%s sentences of training data" % len(self.training))
        path = os.path.join(absbasepath,"../training/") + filename_aiml
        print (path)
        f = open(path, 'r')
        # Read the intents and their sentences
        content = f.readlines()
        content = [x.strip() for x in content]
        for line in content:
            text_state_pair = line.split(",")
            temp_dict = {}
            temp_dict['class'] = text_state_pair[0]
            temp_dict['sentence'] = text_state_pair[1]
            self.aiml_training.append(temp_dict)
        print ("%s sentences of predefined questions training data" % len(self.aiml_training))

    def intent_callback(self, msg):
        aiml_class, aiml_score = self.classify_aiml(msg.data)
        high_class, high_score = self.classify(msg.data)

    def intent(self, req):
        aiml_class, aiml_score = self.classify_aiml(req.question)
        print ('Predefined',aiml_class, aiml_score)
        high_class, high_score = self.classify(req.question)
        print ('General',high_class, high_score)
        if aiml_score >= high_score and high_score < 0.4:
            response = self.aiml_server(aiml_class)
            return response
        elif high_score >= 0.4:
            print (req.question, high_class)
            response = self.nlp_server(req.question, high_class)
            return response.answer
        else:
            response = 'I did not understand your question'
            return response.answer


    def intent_server(self):
        s = rospy.Service('/tiago/speech/classify_intent', srvQuestion, self.intent)

if __name__ == '__main__':
    rospy.init_node('speech_intent')
    rospy.loginfo("SPEECH INTENT: Initializing")
    intent = Intent_Classifier(filename = 'training_nlp.csv', filename_aiml = 'training_predefined.csv')
    rospy.Subscriber('/tiago/speech/classify_intent', String, intent.intent_callback)
    intent.intent_server()
    rospy.spin()
