#!/usr/bin/env python
import rospy
import nltk

from nltk import tag
from nltk.util import ngrams
from collections import OrderedDict

from std_msgs.msg import String
from tum_alle_athome_speech_msgs.srv import srvTTS
from tum_alle_athome_speech_msgs.msg import msgTTSState
from tum_alle_athome_speech_msgs.srv import srvQuestion

import answers

## Class storing basic tokenization methods
#
class ProcessText:

    ## Create a ProcessText class object.
    #
	def __init__(self):
		## Sentence processed
		self.sentence = ''
		## List of punctuation signs
		self.punctuation = ['.', ',', ';', ':']
		## List of verb tags
		self.verbs = ['VB', 'VBD', 'VBG', 'VBN','VBZ','VBP','VERB']
		## List of verb tags
		self.nouns = ['NN', 'NNS', 'NNPS', 'NNP', 'NOUN']
		## Language of the responses
		self.language = 'eng'
		## Extracted command
		self.command = {}

	def to_lower(self, text):
		lowercase = text.lower()
		return lowercase

	def tokenize(self, sentence):
		tokens = nltk.word_tokenize(sentence)
		print ("Tokens:",tokens)
		return tokens

	def tag_words(self, tokens, tag_set = None):
		tagged = nltk.pos_tag(tokens, tagset = tag_set)
		print ("Tags:", tagged)
		return tagged

	def build_tree(self, tagged):
		entities = nltk.chunk.ne_chunk(tagged)
		#print(OrderedDict(entities))
		print ("Entities", entities)
		return entities

	def remove_trash(self, sentence):
		out_sentence = [w for w in sentence if w[1] != 'DT']
		print ("Removed trash", out_sentence)
		return out_sentence

	def get_human_names(self, sentence):
		sentt = nltk.ne_chunk(sentence, binary = False)
		person_list = []
		person = []
		name = ""
		print (sentt)
		for subtree in sentt.subtrees(filter=lambda t: t.label() == 'PERSON'):
			for leaf in subtree.leaves():
				person.append(leaf[0])
				for part in person:
					name += part + ' '
				if name[:-1] not in person_list:
					person_list.append(name[:-1])
				name = ''
			person = []
		print ("Person list: ", person_list)
		return (person_list)

	def get_prp(self, pos_tag):
		operator = []
		for (w1,t1) in pos_tag:
			if (t1.startswith('PRP')):
				print(w1, t1)
				operator.append(w1)
		return operator

	def get_locations(self, pos_tag, names):
		location_start = []
		location_stop = []
		for (w1,t1), (w2,t2) in nltk.bigrams(pos_tag):
			if (t1.startswith('IN') and w1 != 'of' and t2 not in names):
				print(w1, w2)
				location_start.append(w2)
			if (t1.startswith('TO') and t2 not in names):
				print(w1, w2)
				location_stop.append(w2)
		print ('Location start: ', location_start)
		print ('Location stop: ', location_stop)
		return (location_start,location_stop)

	def get_objects(self, pos_tag, locations, names):
		objects = []
		for i in range(len(pos_tag)):
			object = {}
			if (pos_tag[i][1] in self.nouns and pos_tag[i][0] not in names and pos_tag[i][0] not in locations):
				object['object'] = pos_tag[i][0]
				j = 1
				adj = []
				while pos_tag[i-j][1].startswith('JJ'):
					adj.append(pos_tag[i-j][0])
					j = j + 1
				object['properties'] = adj
				j = 1
				if i+j < len(pos_tag) and pos_tag[i+j][0] == 'of':
					while pos_tag[i+j+1][1].startswith('JJ') and j < len(pos_tag):
						j = j + 1
					object['owner'] = pos_tag[i+j+1][0]
				objects.append(object.copy())
		print ('Objects: ', objects)
		return objects

	def divide_sentences(self, text):
		# entities = self.build_tree(self.tag_words(self.tokenize(self.to_lower(text))))
		entities = self.tag_words(self.tokenize(text))
		i = 0
		stop = 0
		sentences = []
		for object in entities:
			key, value = object
			i = i+1
			if (value == 'CC'):
				sentences.append(entities[0:i])
				entities = entities[i+1:]
				i = 0
			if (value in self.verbs):
				if (entities[:i-1] != []):
					sentences.append(entities[:i-1])
					entities = entities[i-1:]
					i = 0
		if (len(entities)):
			sentences.append(entities)
		print ('Sentences: ', sentences)
		return sentences

	def extract_commands(self, sentences):
		commands = []
		command = {}
		# Extract verbs
		for sentence in sentences:
			for k,v in sentence:
				if (v in self.verbs):
					command['action'] = self.to_lower(k)
			command['person'] = list(set().union(self.get_prp(sentence), self.get_human_names(sentence)))
			command['location_object'], command['location_stop'] = self.get_locations(self.remove_trash(sentence), command['person'])
			command['object'] = self.get_objects(self.remove_trash(sentence), list(set().union(command['location_object'], command['location_stop'])), command['person'])
			print ('Command:', command)
			if (command['location_stop'] == []):
				command['location_stop'] = command['person']
			commands.append(command.copy())
		print ('All commands:', commands)
		return commands

	def basic_callback(self, msg):
		self.extract_commands(self.divide_sentences(msg.data))

	def question(self, req):
		response = self.extract_commands(self.divide_sentences(req.question))
		print(response)
		return ('Hello')

	def question_server(self):
		s = rospy.Service('/tiago/speech/process_text', srvQuestion, self.question)

if __name__ == '__main__':
	rospy.init_node('speech_nlp_basic')
	rospy.loginfo("SPEECH NLP: Initializing")
	nlp_basic = ProcessText()
	rospy.Subscriber('/tiago/speech/basic_nlp', String, nlp_basic.basic_callback)
	nlp_basic.question_server()
	rospy.spin()
